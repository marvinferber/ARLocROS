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

import java.util.Arrays;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import sensor_msgs.Image;

public class Utils {

	static public Mat matFromImage(final Image source) throws Exception {
		byte[] imageInBytes = source.getData().array();
		imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
		Mat cvImage = new Mat(source.getHeight(), source.getWidth(), CvType.CV_8UC3);
		cvImage.put(0, 0, imageInBytes);
		return cvImage;
	}

	static public void tresholdContrastBlackWhite(Mat image2, double d) {
		int width = image2.width();
		int height = image2.height();
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++) {
				double[] rgb = image2.get(j, i);
				double[] rgbnew = new double[rgb.length];
				if (rgb[0] + rgb[1] + rgb[2] < d)
					rgbnew[0] = rgbnew[1] = rgbnew[2] = 0.0;
				else
					rgbnew[0] = rgbnew[1] = rgbnew[2] = 255.0;
				image2.put(j, i, rgbnew);
			}
	}
}
