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

import java.util.List;

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
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import rosjava_tf_example.Transformer;
import tf2_msgs.TFMessage;
import visualization_msgs.Marker;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class ARLoc extends AbstractNodeMain {
	private CameraParams camp;
	private Mat image;
	private Mat rvec;
	private MatOfDouble tvec;
	// private Jogl3DView jt;

	private Transformer transformer = new Transformer();
	private String PATTERN_DIR;
	private String MARKER_FRAME_NAME;
	private String CAMERA_FRAME_NAME;
	private String CAMERA_IMAGE_TOPIC;
	private String CAMERA_INFO_TOPIC;
	private String MARKER_CONFIG_FILE;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/imshow");
	}

	public Transformer getTransformer() {
		return transformer;
	}

	public void setTransformer(Transformer transformer) {
		this.transformer = transformer;
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		final Log log = connectedNode.getLog();
		log.info("Reading parameters");
		PATTERN_DIR = connectedNode.getParameterTree().getString("/ARLocROS/pattern_dir");
		MARKER_CONFIG_FILE = connectedNode.getParameterTree().getString("/ARLocROS/marker_config_file");
		MARKER_FRAME_NAME = connectedNode.getParameterTree().getString("/ARLocROS/marker_frame_name");
		CAMERA_FRAME_NAME = connectedNode.getParameterTree().getString("/ARLocROS/camera_frame_name");
		CAMERA_IMAGE_TOPIC = connectedNode.getParameterTree().getString("/ARLocROS/camera_image_topic");
		CAMERA_INFO_TOPIC = connectedNode.getParameterTree().getString("/ARLocROS/camera_info_topic");

		rvec = new Mat(3, 1, CvType.CV_64F);
		tvec = new MatOfDouble(1.0, 1.0, 1.0);
		// start OpenGL outup
		// jt = Jogl3DView.start();

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
		Subscriber<sensor_msgs.Image> subscriberToImage = connectedNode.newSubscriber(CAMERA_IMAGE_TOPIC,
				sensor_msgs.Image._TYPE);
		subscriberToImage.addMessageListener(new MessageListener<sensor_msgs.Image>() {

			@Override
			public void onNewMessage(sensor_msgs.Image message) {
				if (!message.getEncoding().toLowerCase().equals("rgb8")) {
					log.error("Sorry, " + message.getEncoding() + " Image encoding is not supported! EXITING");
					System.exit(-1);
				}
				if (camp != null) {
					try {
						//
						image = Utils.matFromImage(message);
						// image.convertTo(image, -1, 2, 0.0);
						// Imshow.show(image);
						// setup camera matrix and return vectors
						Mat cameraMatrix = new Mat(new Size(3, 3), CvType.CV_32FC1);
						MatOfDouble distCoeffs = new MatOfDouble(new Mat(4, 1, CvType.CV_64FC1));
						CameraParams.getCameraParamas(cameraMatrix, distCoeffs, camp);
						// compute pose
						if (ComputePose.computePose(rvec, tvec, cameraMatrix, distCoeffs, image,
								new Size(camp.width, camp.height), PATTERN_DIR, MARKER_CONFIG_FILE)) {
							// Imshow.show(image);
							// log.info("Pose detected!");
							// jt.showandsafe(rvec, tvec);
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
		Subscriber<sensor_msgs.CameraInfo> subscriberToCameraInfo = connectedNode.newSubscriber(CAMERA_INFO_TOPIC,
				sensor_msgs.CameraInfo._TYPE);
		subscriberToCameraInfo.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {

			@Override
			public void onNewMessage(sensor_msgs.CameraInfo message) {
				if (camp == null) {
					camp = new CameraParams();
					camp.fx = message.getK()[0];
					camp.fy = message.getK()[4];
					;
					camp.cx = message.getK()[2];
					;
					camp.cy = message.getK()[5];
					;
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

		final Publisher<tf2_msgs.TFMessage> publisher1 = connectedNode.newPublisher("tf", tf2_msgs.TFMessage._TYPE);
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
				Mat R = new Mat(3, 3, CvType.CV_32FC1);
				Calib3d.Rodrigues(rvec, R);
				double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				double attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler((float) bankX, (float) headingY, (float) attitudeZ);

				// set information to message
				TFMessage tfmessage = publisher1.newMessage();
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
				posestamped.getHeader().setFrameId(CAMERA_FRAME_NAME);
				posestamped.setChildFrameId(MARKER_FRAME_NAME);
				posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
				// frame_id too
				tfmessage.getTransforms().add(posestamped);
				publisher1.publish(tfmessage);
				// System.exit(0);
			}
		});

		final Publisher<visualization_msgs.Marker> markerpublisher = connectedNode.newPublisher("markers",
				visualization_msgs.Marker._TYPE);
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void loop() throws InterruptedException {

				Thread.sleep(500);
				List<Point3> points3dlist = MarkerConfig.getUnordered3DPointList();
				int i = 0;
				for (Point3 p : points3dlist) {
					Marker markermessage = markerpublisher.newMessage();
					markermessage.getHeader().setFrameId(MARKER_FRAME_NAME);
					markermessage.setId(i);
					i++;
					markermessage.setType(visualization_msgs.Marker.SPHERE);
					markermessage.setAction(visualization_msgs.Marker.ADD);
					double x = p.x;
					markermessage.getPose().getPosition().setX(x);
					double y = p.y;
					markermessage.getPose().getPosition().setY(y);
					double z = p.z;
					markermessage.getPose().getPosition().setZ(z);
					markermessage.getPose().getOrientation().setX(0);
					markermessage.getPose().getOrientation().setY(0);
					markermessage.getPose().getOrientation().setZ(0);
					markermessage.getPose().getOrientation().setW(1);
					markermessage.getScale().setX(0.1);
					markermessage.getScale().setY(0.1);
					markermessage.getScale().setZ(0.1);
					markermessage.getColor().setA(1);
					markermessage.getColor().setR(1);
					markermessage.getColor().setG(0);
					markermessage.getColor().setB(0);

					markerpublisher.publish(markermessage);
				}
			}
		});

		final Publisher<tf2_msgs.TFMessage> tfPublisher_map_to_odom = connectedNode.newPublisher("tf",
				tf2_msgs.TFMessage._TYPE);
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void loop() throws InterruptedException {

				synchronized (tvec) {
					tvec.wait();
				}

				QuaternionHelper q = new QuaternionHelper();

				Mat R = new Mat(3, 3, CvType.CV_32FC1);
				Calib3d.Rodrigues(rvec, R);
				double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				double attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler((float) bankX, (float) headingY, (float) attitudeZ);

				// compute transform map to odom from map to
				// camera_rgb_optical_frame and odom to camera_rgb_optical_frame

				// map to camera_rgb_optical_frame
				Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
				R = R.t();

				// Calib3d.Rodrigues(rvec, R);
				bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler((float) bankX, (float) headingY, (float) attitudeZ);
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
				GraphName sourceFrame = GraphName.of(CAMERA_FRAME_NAME);
				GraphName targetFrame = GraphName.of("odom");
				org.ros.rosjava_geometry.Transform transform_cam_odom = null;
				if (transformer.canTransform(targetFrame, sourceFrame)) {
					transform_cam_odom = transformer.lookupTransform(targetFrame, sourceFrame);

				}
				// multiply results
				org.ros.rosjava_geometry.Transform result = org.ros.rosjava_geometry.Transform.identity();
				result = result.multiply(transform_map_cam);
				result = result.multiply(transform_cam_odom);

				// set information to message
				TFMessage tfmessage = tfPublisher_map_to_odom.newMessage();
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
				tfmessage.getTransforms().add(transformStamped);
				tfPublisher_map_to_odom.publish(tfmessage);
				// System.exit(0);
			}
		});

		// Publish Pose

		final Publisher<geometry_msgs.PoseStamped> publisher = connectedNode.newPublisher("arlocros/pose",
				geometry_msgs.PoseStamped._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void loop() throws InterruptedException {

				synchronized (tvec) {
					tvec.wait();
				}
				QuaternionHelper q = new QuaternionHelper();

				Mat R = new Mat(3, 3, CvType.CV_32FC1);
				Calib3d.Rodrigues(rvec, R);
				double bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				double headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				double attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler((float) bankX, (float) headingY, (float) attitudeZ);

				// compute transform map to odom from map to
				// camera_rgb_optical_frame and odom to camera_rgb_optical_frame

				// map to camera_rgb_optical_frame
				Mat tvec_map_cam = new MatOfDouble(1.0, 1.0, 1.0);
				R = R.t();

				bankX = Math.atan2(-R.get(1, 2)[0], R.get(1, 1)[0]);
				headingY = Math.atan2(-R.get(2, 0)[0], R.get(0, 0)[0]);
				attitudeZ = Math.asin(R.get(1, 0)[0]);
				q.setFromEuler((float) bankX, (float) headingY, (float) attitudeZ);
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
				GraphName sourceFrame = GraphName.of(CAMERA_FRAME_NAME);
				GraphName targetFrame = GraphName.of("base_link");
				org.ros.rosjava_geometry.Transform transform_cam_base = null;
				if (transformer.canTransform(targetFrame, sourceFrame)) {
					transform_cam_base = transformer.lookupTransform(targetFrame, sourceFrame);

				}
				// multiply results
				org.ros.rosjava_geometry.Transform result = org.ros.rosjava_geometry.Transform.identity();
				result = result.multiply(transform_map_cam);
				result = result.multiply(transform_cam_base);
				
				// set information to message
				geometry_msgs.PoseStamped posestamped = publisher.newMessage();
				Pose pose = posestamped.getPose();
				Quaternion orientation = pose.getOrientation();
				Point point = pose.getPosition();
				
				point.setX(result.getTranslation().getX());

				point.setY(result.getTranslation().getY());

				point.setZ(result.getTranslation().getZ());

				orientation.setW(result.getRotationAndScale().getW());
				orientation.setX(result.getRotationAndScale().getX());
				orientation.setY(result.getRotationAndScale().getY());
				orientation.setZ(result.getRotationAndScale().getZ());

				
				// frame_id too
				posestamped.getHeader().setFrameId("map");
				posestamped.getHeader().setStamp(connectedNode.getCurrentTime());
				publisher.publish(posestamped);


			}
		});

	}

}
