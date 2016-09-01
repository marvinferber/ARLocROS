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

import java.io.FileNotFoundException;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.annotation.Nullable;

import org.apache.commons.logging.Log;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Optional;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import jp.nyatla.nyartoolkit.core.NyARException;
import sensor_msgs.CameraInfo;
import tf2_msgs.TFMessage;
import visualization_msgs.Marker;

/**
 * Main Class of the ARLocROS Node setting up publishers and subscribers. Note
 * the TF lookup implementation based on Transformer by Lorenz Moesenlechner.
 */
public class ARLoc extends AbstractNodeMain {

	private static final Logger logger = LoggerFactory.getLogger(ARLoc.class);

	private CameraParams camp;
	private Mat image;
	private Mat rvec;
	private MatOfDouble tvec;

	@Nullable
	private Parameter parameter;
	private MarkerConfig markerConfig;
	protected org.ros.rosjava_geometry.Transform lastPose;
	protected Time lastPoseTimestamp;
	protected Time lastOdomTimestamp;

	protected boolean smoothing = true;

	protected org.ros.rosjava_geometry.Transform oldOdomTransform;

	protected org.ros.rosjava_geometry.Transform odomDelta;
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

		camp = getCameraInfo(connectedNode, parameter);

		// start to listen to transform messages in /tf in order to feed the
		// Transformer and lookup transforms
		final TransformationService transformationService = TransformationService.create(connectedNode);

		// init pose publisher
		final Publisher<geometry_msgs.PoseStamped> posePublisher = connectedNode.newPublisher(parameter.poseTopicName(),
				geometry_msgs.PoseStamped._TYPE);

		// Subscribe to odom
		Subscriber<nav_msgs.Odometry> subscriberToOdom = connectedNode.newSubscriber("/bebop/odom",
				nav_msgs.Odometry._TYPE);
		subscriberToOdom.addMessageListener(new MessageListener<nav_msgs.Odometry>() {

			@Override
			public void onNewMessage(nav_msgs.Odometry odom) {
				// set last odom value when invoked the first time, then wait
				// for second message
				if (oldOdomTransform == null) {
					oldOdomTransform = org.ros.rosjava_geometry.Transform.fromPoseMessage(odom.getPose().getPose());
					lastOdomTimestamp = connectedNode.getCurrentTime();
				} else {
					// convert to current transform
					org.ros.rosjava_geometry.Transform newOdomTransform = org.ros.rosjava_geometry.Transform
							.fromPoseMessage(odom.getPose().getPose());
					// compute delta
					odomDelta = oldOdomTransform.invert().multiply(newOdomTransform);
					// save for current pose for next message
					oldOdomTransform = newOdomTransform;
					// do something when odom is newer than pose
					if (lastPoseTimestamp != null && lastOdomTimestamp.compareTo(lastPoseTimestamp) >= 0) {
						log.info("Pose outdateted --> Odom newer!");
						org.ros.rosjava_geometry.Transform current_pose = lastPose.multiply(odomDelta);
						lastPose = current_pose;
						lastPoseTimestamp = connectedNode.getCurrentTime();
						// set information to message
						geometry_msgs.PoseStamped posestamped = posePublisher.newMessage();
						current_pose.toPoseStampedMessage(GraphName.of("map"), connectedNode.getCurrentTime(),
								posestamped);
						posePublisher.publish(posestamped);
					}
					lastOdomTimestamp = connectedNode.getCurrentTime();
				}
			}
		});

		// Subscribe to Image
		Subscriber<sensor_msgs.Image> subscriberToImage = connectedNode.newSubscriber(parameter.cameraImageTopic(),
				sensor_msgs.Image._TYPE);

		ComputePose computePose = null;
		try {
			final Mat cameraMatrix = CameraParams.getCameraMatrix(camp);
			final MatOfDouble distCoeffs = CameraParams.getDistCoeffs(camp);
			computePose = ComputePose.create(markerConfig, new Size(camp.width(), camp.height()), cameraMatrix,
					distCoeffs, this.parameter.visualization());
		} catch (NyARException e) {
			logger.info("Cannot initialize ComputePose", e);
		} catch (FileNotFoundException e) {
			logger.info("Cannot find file when initialize ComputePose", e);
		}
		final ComputePose poseProcessor = computePose;
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
						// Utils.tresholdContrastBlackWhite(image, 600);
						Imgproc.threshold(image, image, 127, 255, Imgproc.THRESH_BINARY);
						// Mat cannyimg = new Mat(image.height(), image.width(),
						// CvType.CV_8UC3);
						// Imgproc.Canny(image, cannyimg, 10, 100);
						// Imshow.show(cannyimg);

						// image.convertTo(image, -1, 1.5, 0);
						// setup camera matrix and return vectors
						// compute pose
						if (poseProcessor.computePose(rvec, tvec, image)) {
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
					// patterntSize
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
				if (transformationService.canTransform(targetFrame, sourceFrame)) {
					try {
						transform_cam_odom = transformationService.lookupTransform(targetFrame, sourceFrame);
					} catch (Exception e) {
						e.printStackTrace();
						log.info("Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "odom! "
								+ "However, " + "will continue..");
						return;
					}
				} else {
					log.info("Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "odom! "
							+ "However, will " + "continue..");
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
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void loop() throws InterruptedException {

				// since this is an infinite loop, wait here to be notified if
				// new image was processed
				synchronized (tvec) {
					tvec.wait();
				}
				final QuaternionHelper q = new QuaternionHelper();

				// convert rotation vector result of solvepnp to rotation matrix
				Mat R = new Mat(3, 3, CvType.CV_32FC1);
				Calib3d.Rodrigues(rvec, R);
				// see publishers before for documentation
				final Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
				R = R.t();
				final double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				final double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				final double attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler(bankX, headingY, attitudeZ);
				Core.multiply(R, new Scalar(-1), R);
				Core.gemm(R, tvec, 1, new Mat(), 0, tvec_map_cam, 0);
				final org.ros.rosjava_geometry.Quaternion rotation = new org.ros.rosjava_geometry.Quaternion(q.getX(),
						q.getY(), q.getZ(), q.getW());
				final double x = tvec_map_cam.get(0, 0)[0];
				final double y = tvec_map_cam.get(1, 0)[0];
				final double z = tvec_map_cam.get(2, 0)[0];

				final org.ros.rosjava_geometry.Vector3 translation = new org.ros.rosjava_geometry.Vector3(x, y, z);
				final org.ros.rosjava_geometry.Transform transform_map_cam = new org.ros.rosjava_geometry.Transform(
						translation, rotation);

				// odom to camera_rgb_optical_frame
				final GraphName sourceFrame = GraphName.of(parameter.cameraFrameName());
				final GraphName targetFrame = GraphName.of("base_link");
				org.ros.rosjava_geometry.Transform transform_cam_base = null;

				if (transformationService.canTransform(targetFrame, sourceFrame)) {
					try {
						transform_cam_base = transformationService.lookupTransform(targetFrame, sourceFrame);
					} catch (Exception e) {
						e.printStackTrace();
						log.info("Cloud not get transformation from " + parameter.cameraFrameName() + " to "
								+ "base_link! " + "However, will continue..");
						// cancel this loop..no result can be computed
						return;
					}
				} else {
					log.info("Cloud not get transformation from " + parameter.cameraFrameName() + " to " + "base_link!"
							+ " However, " + "will continue..");
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
					double maxspeed = 10;
					boolean goodpose = false;
					// if (current_pose != null && current_timestamp != null) {
					if (lastPose != null && lastPoseTimestamp != null) {
						// check speed of movement between last and current pose
						double distance = PoseCompare.distance(current_pose, lastPose);
						double timedelta = PoseCompare.timedelta(current_timestamp, lastPoseTimestamp);
						if ((distance / timedelta) < maxspeed) {
							if (smoothing) {
								double xold = lastPose.getTranslation().getX();
								double yold = lastPose.getTranslation().getY();
								double zold = lastPose.getTranslation().getZ();
								double xnew = current_pose.getTranslation().getX();
								double ynew = current_pose.getTranslation().getY();
								double znew = current_pose.getTranslation().getZ();
								final org.ros.rosjava_geometry.Vector3 smoothTranslation = new org.ros.rosjava_geometry.Vector3(
										(xold * 2 + xnew) / 3, (yold * 2 + ynew) / 3, (zold * 2 + znew) / 3);
								current_pose = new org.ros.rosjava_geometry.Transform(smoothTranslation,
										current_pose.getRotationAndScale());
							}
							lastPose = current_pose;
							lastPoseTimestamp = current_timestamp;
							goodpose = true;
						} else {
							log.debug("distance " + distance + " time: " + timedelta + " --> Pose rejected");
						}

					} else {
						lastPose = current_pose;
						lastPoseTimestamp = current_timestamp;
					}
					// }
					// bad pose rejection
					if (!goodpose) {
						return;
					}
				}
				log.info("Odom outdateted --> Pose newer!");
				// set information to message
				geometry_msgs.PoseStamped posestamped = posePublisher.newMessage();
				current_pose.toPoseStampedMessage(GraphName.of("map"), connectedNode.getCurrentTime(), posestamped);
				posePublisher.publish(posestamped);

			}
		});

	}

	private static CameraParams getCameraInfo(ConnectedNode connectedNode, Parameter parameter) {// Subscribe
																									// to
																									// camera
																									// info
		Subscriber<CameraInfo> subscriberToCameraInfo = connectedNode.newSubscriber(parameter.cameraInfoTopic(),
				CameraInfo._TYPE);
		final CameraInfoService cameraInfoService = CameraInfoService.create(subscriberToCameraInfo);
		Optional<CameraParams> cameraParamsOptional = cameraInfoService.getCameraParams();
		while (!cameraParamsOptional.isPresent()) {
			// we're not gonna do anything before getting the camera info
			try {
				TimeUnit.MILLISECONDS.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			cameraParamsOptional = cameraInfoService.getCameraParams();
		}
		return cameraParamsOptional.get();
	}

	/**
	 * @return
	 */
	public static Log getLog() {
		return log;

	}

}
