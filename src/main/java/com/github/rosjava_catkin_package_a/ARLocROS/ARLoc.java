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
import org.opencv.core.Size;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
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
	//private Jogl3DView jt;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/imshow");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		final Log log = connectedNode.getLog();
		rvec = new Mat(3, 1, CvType.CV_64F);
		tvec = new MatOfDouble(1.0, 1.0, 1.0);
		// start OpenGL outup
		//jt = Jogl3DView.start();
		// Subscribe to Image
		Subscriber<sensor_msgs.Image> subscriberToImage = connectedNode.newSubscriber("camera/rgb/image_raw",
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
						image.convertTo(image, -1, 2, 0.0);
						// Imshow.show(image);
						// setup camera matrix and return vectors
						Mat cameraMatrix = new Mat(new Size(3, 3), CvType.CV_32FC1);
						MatOfDouble distCoeffs = new MatOfDouble(new Mat(4, 1, CvType.CV_64FC1));
						CameraParams.getCameraParamas(cameraMatrix, distCoeffs, camp);
						// compute pose
						if (ComputePose.computePose(rvec, tvec, cameraMatrix, distCoeffs, image,
								new Size(camp.width, camp.height))) {
							// Imshow.show(image);
							// log.info(rvec.dump() + " " + tvec.dump());
							//jt.showandsafe(rvec, tvec);
							synchronized (tvec) {
								tvec.notify();
							}
						}

					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			}
		});
		// Subscribe to camera info
		Subscriber<sensor_msgs.CameraInfo> subscriberToCameraInfo = connectedNode
				.newSubscriber("camera/rgb/camera_info", sensor_msgs.CameraInfo._TYPE);
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
		// Publish Pose

		// final Publisher<geometry_msgs.PoseStamped> publisher =
		// connectedNode.newPublisher("mypose",
		// geometry_msgs.PoseStamped._TYPE);
		//
		// connectedNode.executeCancellableLoop(new CancellableLoop() {
		//
		// @Override
		// protected void loop() throws InterruptedException {
		//
		// synchronized (tvec) {
		// tvec.wait();
		// }
		// // compute translation
		// // Mat R = new Mat(3, 3, CvType.CV_32FC1);
		// // Calib3d.Rodrigues(rvec, R);
		// // R = R.t();
		// // Core.multiply(R, new Scalar(-1), R);
		// // Core.gemm(R, tvec, 1, new Mat(), 0, tvec, 0);
		// // compute rotation
		//
		// QuaternionHelper q = new QuaternionHelper();
		// float bankX = (float) rvec.get(0, 0)[0];
		// float headingY = (float) rvec.get(1, 0)[0];
		// float attitudeZ = (float) rvec.get(2, 0)[0];
		// q.setFromEuler(bankX, headingY, attitudeZ);
		// System.out.println("Roll " + Math.toDegrees(rvec.get(0, 0)[0]) + "
		// Pitch "
		// + Math.toDegrees(rvec.get(1, 0)[0]) + " Yaw " +
		// Math.toDegrees(rvec.get(2, 0)[0]));
		// // set information to message
		// geometry_msgs.PoseStamped posestamped = publisher.newMessage();
		// Pose pose = posestamped.getPose();
		// Quaternion orientation = pose.getOrientation();
		// Point point = pose.getPosition();
		// // point.setX(tvec.get(0, 0)[0]);
		// // point.setY(tvec.get(1, 0)[0]);
		// // point.setZ(tvec.get(2, 0)[0]);
		// point.setX(0);
		// point.setY(0);
		// point.setZ(0);
		// orientation.setW(q.getW());
		// orientation.setX(q.getX());
		// orientation.setY(q.getY());
		// orientation.setZ(q.getZ());
		// // frame_id too
		// posestamped.getHeader().setFrameId("base_link");
		//
		// publisher.publish(posestamped);
		//
		// }
		// });

		final Publisher<tf2_msgs.TFMessage> publisher1 = connectedNode.newPublisher("tf", tf2_msgs.TFMessage._TYPE);
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void loop() throws InterruptedException {

				synchronized (tvec) {
					tvec.wait();
				}

				QuaternionHelper q = new QuaternionHelper();

				/*
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
				posestamped.getHeader().setFrameId("camera_rgb_optical_frame");
				posestamped.setChildFrameId("marker");
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
				List<Point3> points3dlist = MarkerConfig.create3dpointlist();
				int i = 0;
				for (Point3 p : points3dlist) {
					Marker markermessage = markerpublisher.newMessage();
					markermessage.getHeader().setFrameId("marker");
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

	}

}
