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

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class CameraParams {

	protected double fx;
	protected double fy;
	protected double cx;
	protected double cy;
	protected double k1;
	protected double k2;
	protected double p1;
	protected double p2;
	protected int width;
	protected int height;
	protected String frame_id;

	public static void getCameraParamas(Mat cameraMatrix, MatOfDouble distCoeffs2, CameraParams camps) {
		cameraMatrix.put(0, 0, camps.fx);
		cameraMatrix.put(0, 1, 0);
		cameraMatrix.put(0, 2, camps.cx);
		cameraMatrix.put(1, 0, 0);
		cameraMatrix.put(1, 1, camps.fy);
		cameraMatrix.put(1, 2, camps.cy);
		cameraMatrix.put(2, 0, 0);
		cameraMatrix.put(2, 1, 0);
		cameraMatrix.put(2, 2, 1);
		distCoeffs2.put(0, 0, camps.k1);
		distCoeffs2.put(1, 0, camps.k2);
		distCoeffs2.put(2, 0, camps.p1);
		distCoeffs2.put(3, 0, camps.p2);
	}

}
