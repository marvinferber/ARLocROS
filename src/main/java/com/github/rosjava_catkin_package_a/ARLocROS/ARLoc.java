/*
 * Copyright (C) 2016 Marvin Ferber.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava_catkin_package_a.ARLocROS;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import org.apache.commons.logging.Log;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import rosjava_tf_example.Transformer;
import tf2_msgs.TFMessage;
import visualization_msgs.Marker;

import javax.annotation.Nullable;
import java.util.List;

/**
 * Main Class of the ARLocROS Node setting up publishers and subscribers. Note
 * the TF lookup implementation based on Transformer by Lorenz Moesenlechner.
 */
public class ARLoc extends AbstractNodeMain {
    private CameraParams camp;
    private Mat image;
    private Mat rvec;
    private MatOfDouble tvec;

    private Transformer transformer = Transformer.create();
    @Nullable private Parameter parameter;
    private MarkerConfig markerConfig;
    protected org.ros.rosjava_geometry.Transform last_pose;
    protected Time last_timestamp;
    private static Log log;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava/imshow");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // load OpenCV shared library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // read configuration variables from the ROS Runtime (configured in the
        // launch file)
        log = connectedNode.getLog();
        log.info("Reading parameters");
        this.parameter = Parameter.createFromParameterTree(connectedNode.getParameterTree());

        // Read Marker Config
        markerConfig = MarkerConfig.createFromConfig(parameter.markerConfigFile(), parameter.patternDirectory());

        // setup rotation vector and translation vector storing output of the
        // localization
        rvec = new Mat(3, 1, CvType.CV_64F);
        tvec = new MatOfDouble(1.0, 1.0, 1.0);

        // start to listen to transform messages in /tf in order to feed the
        // Transformer and lookup transforms
        transformer.setPrefix(GraphName.of(connectedNode.getParameterTree().getString("~tf_prefix", "")));
        Subscriber<TFMessage> tfSubscriber = connectedNode.newSubscriber(GraphName.of("tf"), tf2_msgs.TFMessage._TYPE);
        tfSubscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
            @Override
            public void onNewMessage(TFMessage message) {
                for (TransformStamped transform : message.getTransforms()) {
                    transformer.updateTransform(transform);
                }
            }
        });

        // Subscribe to Image
        Subscriber<sensor_msgs.Image> subscriberToImage = connectedNode.newSubscriber(parameter.cameraImageTopic(),
                sensor_msgs.Image._TYPE);
        subscriberToImage.addMessageListener(new MessageListener<sensor_msgs.Image>() {

            @Override
            public void onNewMessage(sensor_msgs.Image message) {
                //
                if (!message.getEncoding().toLowerCase().equals("rgb8")) {
                    log.error("Sorry, " + message.getEncoding() + " Image encoding is not supported! EXITING");
                    System.exit(-1);
                }
                if (camp != null) {
                    try {
                        //
                        image = Utils.matFromImage(message);
                        // uncomment to add more contrast to the image
                        // image.convertTo(image, -1, 2, 0.0);
                        // setup camera matrix and return vectors
                        Mat cameraMatrix = new Mat(new Size(3, 3), CvType.CV_32FC1);
                        MatOfDouble distCoeffs = new MatOfDouble(new Mat(4, 1, CvType.CV_64FC1));
                        CameraParams.getCameraParamas(cameraMatrix, distCoeffs, camp);
                        // compute pose
                        if (ComputePose.computePose(rvec, tvec, cameraMatrix, distCoeffs, image,
                                new Size(camp.width, camp.height), markerConfig)) {
                            // notify publisher threads (pose and tf, see below)
                            synchronized (tvec) {
                                tvec.notifyAll();
                            }
                        }

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        // Subscribe to camera info
        Subscriber<sensor_msgs.CameraInfo> subscriberToCameraInfo = connectedNode.newSubscriber(parameter.cameraInfoTopic(),
                sensor_msgs.CameraInfo._TYPE);
        subscriberToCameraInfo.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {

            @Override
            public void onNewMessage(sensor_msgs.CameraInfo message) {
                // capture the camera intrinsics once to be used later by solvepnp
                if (camp == null) {
                    camp = new CameraParams();
                    camp.fx = message.getK()[0];
                    camp.fy = message.getK()[4];
                    camp.cx = message.getK()[2];
                    camp.cy = message.getK()[5];
                    camp.k1 = message.getD()[0];
                    camp.k2 = message.getD()[1];
                    camp.p1 = message.getD()[2];
                    camp.p2 = message.getD()[3];
                    camp.width = message.getWidth();
                    camp.height = message.getHeight();
                    camp.frame_id = message.getHeader().getFrameId();
                    log.info("Setting up camera parameters");
                }
            }
        });
        log.info("Setting up camera parameters");

        // publish tf CAMERA_FRAME_NAME --> MARKER_FRAME_NAME
        final Publisher<tf2_msgs.TFMessage> tfPublisherCamToMarker = connectedNode.newPublisher("tf",
                tf2_msgs.TFMessage._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void loop() throws InterruptedException {

                synchronized (tvec) {
                    tvec.wait();
                }

                QuaternionHelper q = new QuaternionHelper();

				/*
                 * http://euclideanspace.com/maths/geometry/rotations/
				 * conversions/matrixToEuler/index.htm
				 * http://stackoverflow.com/questions/12933284/rodrigues-into-
				 * eulerangles-and-vice-versa
				 * 
				 * heading = atan2(-m20,m00) attitude = asin(m10) bank =
				 * atan2(-m12,m11)
				 */
                // convert output rotation vector rvec to rotation matrix R
                Mat R = new Mat(3, 3, CvType.CV_32FC1);
                Calib3d.Rodrigues(rvec, R);
                // get rotations around X,Y,Z from rotation matrix R
                double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
                double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
                double attitudeZ = Math.asin(R.get(1, 0)[0]);
                // convert Euler angles to quarternion
                q.setFromEuler(bankX, headingY, attitudeZ);

                // set information to message
                TFMessage tfmessage = tfPublisherCamToMarker.newMessage();
                TransformStamped posestamped = connectedNode.getTopicMessageFactory()
                        .newFromType(geometry_msgs.TransformStamped._TYPE);
                Transform transform = posestamped.getTransform();

                Quaternion orientation = transform.getRotation();
                Vector3 point = transform.getTranslation();
                point.setX(tvec.get(0, 0)[0]);
                point.setY(tvec.get(1, 0)[0]);
                point.setZ(tvec.get(2, 0)[0]);

                orientation.setW(q.getW());
                orientation.setX(q.getX());
                orientation.setY(q.getY());
                orientation.setZ(q.getZ());
                posestamped.getHeader().setFrameId(parameter.cameraFrameName());
                posestamped.setChildFrameId(parameter.markerFrameName());
                posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
                // frame_id too
                tfmessage.getTransforms().add(posestamped);
                tfPublisherCamToMarker.publish(tfmessage);
            }
        });

        // publish Markers
        final Publisher<visualization_msgs.Marker> markerPublisher = connectedNode.newPublisher("markers",
                visualization_msgs.Marker._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void loop() throws InterruptedException {
                // publish markers every 500ms
                Thread.sleep(500);
                // get marker points from markerConfig, each marker has 4
                // vertices
                List<Point3> points3dlist = markerConfig.getUnordered3DPointList();
                int i = 0;
                for (Point3 p : points3dlist) {
                    Marker markermessage = markerPublisher.newMessage();
                    // FIXME If the markers are published into an existing frame
                    // (e.g. map or odom) the node will consume very high CPU
                    // and will fail after a short time. The markers are
                    // probably published in the wrong way.
                    markermessage.getHeader().setFrameId(parameter.markerFrameName());
                    markermessage.setId(i);
                    i++;
                    markermessage.setType(visualization_msgs.Marker.SPHERE);
                    markermessage.setAction(visualization_msgs.Marker.ADD);
                    // position
                    double x = p.x;
                    markermessage.getPose().getPosition().setX(x);
                    double y = p.y;
                    markermessage.getPose().getPosition().setY(y);
                    double z = p.z;
                    markermessage.getPose().getPosition().setZ(z);
                    // orientation
                    markermessage.getPose().getOrientation().setX(0);
                    markermessage.getPose().getOrientation().setY(0);
                    markermessage.getPose().getOrientation().setZ(0);
                    markermessage.getPose().getOrientation().setW(1);
                    // size
                    markermessage.getScale().setX(0.1);
                    markermessage.getScale().setY(0.1);
                    markermessage.getScale().setZ(0.1);
                    // color
                    markermessage.getColor().setA(1);
                    markermessage.getColor().setR(1);
                    markermessage.getColor().setG(0);
                    markermessage.getColor().setB(0);

                    markerPublisher.publish(markermessage);
                }
            }
        });

        // publish tf map --> odom
        final Publisher<tf2_msgs.TFMessage> tfPublisherMapToOdom = connectedNode.newPublisher("tf",
                tf2_msgs.TFMessage._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void loop() throws InterruptedException {

                // since this is an infinite loop, wait to be notified if new
                // image was processed
                synchronized (tvec) {
                    tvec.wait();
                }

                // compute transform map to odom from map to
                // camera_rgb_optical_frame and odom to camera_rgb_optical_frame

                // map to camera_rgb_optical_frame
                Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
                QuaternionHelper q = new QuaternionHelper();
                // get rotation matrix R from solvepnp output rotation vector
                // rvec
                Mat R = new Mat(3, 3, CvType.CV_32FC1);
                Calib3d.Rodrigues(rvec, R);
                // transpose R, because we need the transformation from
                // world(map) to camera
                R = R.t();
                // get rotation around X,Y,Z from R in radiants
                double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
                double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
                double attitudeZ = Math.asin(R.get(1, 0)[0]);
                q.setFromEuler(bankX, headingY, attitudeZ);
                // compute translation vector from world (map) to cam
                // tvec_map_cam
                Core.multiply(R, new Scalar(-1), R); // R=-R
                Core.gemm(R, tvec, 1, new Mat(), 0, tvec_map_cam, 0); // tvec_map_cam=R*tvec

                org.ros.rosjava_geometry.Quaternion rotation = new org.ros.rosjava_geometry.Quaternion(q.getX(),
                        q.getY(), q.getZ(), q.getW());
                double x = tvec_map_cam.get(0, 0)[0];
                double y = tvec_map_cam.get(1, 0)[0];
                double z = tvec_map_cam.get(2, 0)[0];
                // create a Transform Object that hold the transform map to cam
                org.ros.rosjava_geometry.Vector3 translation = new org.ros.rosjava_geometry.Vector3(x, y, z);
                org.ros.rosjava_geometry.Transform transform_map_cam = new org.ros.rosjava_geometry.Transform(
                        translation, rotation);

                // odom to camera_rgb_optical_frame
                GraphName sourceFrame = GraphName.of(parameter.cameraFrameName());
                GraphName targetFrame = GraphName.of("odom");
                org.ros.rosjava_geometry.Transform transform_cam_odom = null;
                if (transformer.canTransform(targetFrame, sourceFrame)) {
                    try {
                        transform_cam_odom = transformer.lookupTransform(targetFrame, sourceFrame);
                    } catch (Exception e) {
                        e.printStackTrace();
                        log.info("Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "odom! However, " +
                                "will continue..");
                        return;
                    }
                } else {
                    log.info(
                            "Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "odom! However, will " +
                                    "continue..");
                    // cancel this loop..no result can be computed
                    return;
                }
                // multiply results
                org.ros.rosjava_geometry.Transform result = org.ros.rosjava_geometry.Transform.identity();
                result = result.multiply(transform_map_cam);
                result = result.multiply(transform_cam_odom);

                // set information to ROS message
                TFMessage tfMessage = tfPublisherMapToOdom.newMessage();
                TransformStamped transformStamped = connectedNode.getTopicMessageFactory()
                        .newFromType(geometry_msgs.TransformStamped._TYPE);
                Transform transform = transformStamped.getTransform();

                Quaternion orientation = transform.getRotation();
                Vector3 vector = transform.getTranslation();
                vector.setX(result.getTranslation().getX());
                vector.setY(result.getTranslation().getY());
                vector.setZ(result.getTranslation().getZ());

                orientation.setW(result.getRotationAndScale().getW());
                orientation.setX(result.getRotationAndScale().getX());
                orientation.setY(result.getRotationAndScale().getY());
                orientation.setZ(result.getRotationAndScale().getZ());
                transformStamped.getHeader().setFrameId("map");
                transformStamped.setChildFrameId("odom");
                transformStamped.getHeader().setStamp(connectedNode.getCurrentTime());
                // frame_id too
                tfMessage.getTransforms().add(transformStamped);
                tfPublisherMapToOdom.publish(tfMessage);
                // System.exit(0);
            }
        });

        // Publish Pose

        final Publisher<geometry_msgs.PoseStamped> posePublisher = connectedNode.newPublisher("arlocros/pose",
                geometry_msgs.PoseStamped._TYPE);

        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void loop() throws InterruptedException {

                // since this is an infinite loop, wait here to be notified if
                // new image was processed
                synchronized (tvec) {
                    tvec.wait();
                }
                QuaternionHelper q = new QuaternionHelper();

                // convert rotation vector result of solvepnp to rotation matrix
                Mat R = new Mat(3, 3, CvType.CV_32FC1);
                Calib3d.Rodrigues(rvec, R);
                // see publishers before for documentation
                Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
                R = R.t();
                double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
                double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
                double attitudeZ = Math.asin(R.get(1, 0)[0]);
                q.setFromEuler(bankX, headingY, attitudeZ);
                Core.multiply(R, new Scalar(-1), R);
                Core.gemm(R, tvec, 1, new Mat(), 0, tvec_map_cam, 0);
                org.ros.rosjava_geometry.Quaternion rotation = new org.ros.rosjava_geometry.Quaternion(q.getX(),
                        q.getY(), q.getZ(), q.getW());
                double x = tvec_map_cam.get(0, 0)[0];
                double y = tvec_map_cam.get(1, 0)[0];
                double z = tvec_map_cam.get(2, 0)[0];

                org.ros.rosjava_geometry.Vector3 translation = new org.ros.rosjava_geometry.Vector3(x, y, z);
                org.ros.rosjava_geometry.Transform transform_map_cam = new org.ros.rosjava_geometry.Transform(
                        translation, rotation);

                // odom to camera_rgb_optical_frame
                GraphName sourceFrame = GraphName.of(parameter.cameraFrameName());
                GraphName targetFrame = GraphName.of("base_link");
                org.ros.rosjava_geometry.Transform transform_cam_base = null;

                if (transformer.canTransform(targetFrame, sourceFrame)) {
                    try {
                        transform_cam_base = transformer.lookupTransform(targetFrame, sourceFrame);
                    } catch (Exception e) {
                        e.printStackTrace();
                        log.info(
                                "Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "base_link! " +
                                        "However, will continue..");
                        // cancel this loop..no result can be computed
                        return;
                    }
                } else {
                    log.info(
                            "Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "base_link! However, " +
                                    "will continue..");
                    // cancel this loop..no result can be computed
                    return;
                }

                // multiply results
                org.ros.rosjava_geometry.Transform current_pose = org.ros.rosjava_geometry.Transform.identity();
                current_pose = current_pose.multiply(transform_map_cam);
                current_pose = current_pose.multiply(transform_cam_base);

                // check for plausibility of the pose by checking if movement
                // exceeds max speed (defined) of the robot
                if (parameter.badPoseReject()) {
                    Time current_timestamp = connectedNode.getCurrentTime();
                    // TODO Unfortunately, we do not have the tf timestamp at
                    // hand here. So we can only use the current timestamp.
                    double maxspeed = 5;
                    boolean goodpose = false;
                    // if (current_pose != null && current_timestamp != null) {
                    if (last_pose != null && last_timestamp != null) {
                        // check speed of movement between last and current pose
                        double distance = PoseCompare.distance(current_pose, last_pose);
                        double timedelta = PoseCompare.timedelta(current_timestamp, last_timestamp);
                        if ((distance / timedelta) < maxspeed) {
                            last_pose = current_pose;
                            last_timestamp = current_timestamp;
                            goodpose = true;
                        } else {
                            log.info("distance " + distance + " time: " + timedelta + " --> Pose rejected");
                        }

                    } else {
                        last_pose = current_pose;
                        last_timestamp = current_timestamp;
                    }
                    // }
                    // bad pose rejection
                    if (!goodpose) {
                        return;
                    }
                }

                // set information to message
                geometry_msgs.PoseStamped posestamped = posePublisher.newMessage();
                Pose pose = posestamped.getPose();
                Quaternion orientation = pose.getOrientation();
                Point point = pose.getPosition();

                point.setX(current_pose.getTranslation().getX());

                point.setY(current_pose.getTranslation().getY());

                point.setZ(current_pose.getTranslation().getZ());

                orientation.setW(current_pose.getRotationAndScale().getW());
                orientation.setX(current_pose.getRotationAndScale().getX());
                orientation.setY(current_pose.getRotationAndScale().getY());
                orientation.setZ(current_pose.getRotationAndScale().getZ());

                // frame_id too
                posestamped.getHeader().setFrameId("map");
                posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
                posePublisher.publish(posestamped);

            }
        });

    }

    /**
     * @return
     */
    public static Log getLog() {
        return log;

    }

}
