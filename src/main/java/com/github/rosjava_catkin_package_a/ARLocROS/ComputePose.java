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

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;

import jp.nyatla.nyartoolkit.core.NyARCode;
import jp.nyatla.nyartoolkit.core.NyARException;
import jp.nyatla.nyartoolkit.core.param.NyARCameraDistortionFactorV2;
import jp.nyatla.nyartoolkit.core.param.NyARParam;
import jp.nyatla.nyartoolkit.core.param.NyARPerspectiveProjectionMatrix;
import jp.nyatla.nyartoolkit.core.raster.rgb.INyARRgbRaster;
import jp.nyatla.nyartoolkit.core.raster.rgb.NyARRgbRaster;
import jp.nyatla.nyartoolkit.core.types.NyARBufferType;
import jp.nyatla.nyartoolkit.core.types.NyARIntPoint2d;
import jp.nyatla.nyartoolkit.core.types.NyARIntSize;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystem;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystemConfig;
import jp.nyatla.nyartoolkit.markersystem.NyARSensor;

/**
 * Static class that contains the pose computation from multiple AR marker
 * system using NyARToolkit Java library.
 * http://nyatla.jp/nyartoolkit/wp/?page_id=198
 *
 */
public class ComputePose {

	private static List<String> markerPatterns;
	private static Map<Integer, String> patternmap;
	private static NyARIntSize i_screen_size;
	private static NyARPerspectiveProjectionMatrix i_projection_mat;
	private static NyARCameraDistortionFactorV2 i_dist_factor;
	private static NyARParam i_param;
	private static NyARMarkerSystemConfig i_config;
	private static NyARMarkerSystem markerSystemState;
	private static NyARSensor cameraSensorWrapper;
	private static int[] ids;

	/**
	 * @param rvec
	 * @param tvec
	 * @param cameraMatrix
	 * @param distCoeffs
	 * @param image2
	 * @param size
	 * @param markerConfig
	 * @return if the localization was successful
	 * @throws NyARException
	 * @throws FileNotFoundException
	 */
	public static boolean computePose(Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs, Mat image2,
			Size size, MarkerConfig markerConfig) throws NyARException, FileNotFoundException {

		// only load the whole configuration once
		if (markerPatterns == null) {
			// get pattern files from marker config
			markerPatterns = markerConfig.getPatternFileList();
			// create hashmap of correspondences between id and file/pattern
			// name
			patternmap = new HashMap<>();
			// create and load camera specific classes
			i_screen_size = new NyARIntSize((int) size.width, (int) size.height);
			i_projection_mat = new NyARPerspectiveProjectionMatrix();
			i_dist_factor = new NyARCameraDistortionFactorV2();
			i_param = new NyARParam(i_screen_size, i_projection_mat, i_dist_factor);

			// create new marker system configuration
			i_config = new NyARMarkerSystemConfig(i_param);
			markerSystemState = new NyARMarkerSystem(i_config);
			// Create wrapper that passes cam pictures to marker system
			cameraSensorWrapper = new NyARSensor(i_screen_size);
			ids = new int[markerPatterns.size()];
			for (int i = 0; i < markerPatterns.size(); i++) {
				// create marker description from pattern file and add to marker
				// system
				NyARCode code = NyARCode.createFromARPattFile(new FileInputStream(markerPatterns.get(i)), 16, 16);
				ids[i] = markerSystemState.addARMarker(code, 25, markerConfig.getMarkerSize());
				patternmap.put(ids[i], markerPatterns.get(i));
			}
		}

		// convert image to NyAR style for processing
		INyARRgbRaster imageRaster = NyARImageHelper.createFromMat(image2);
		cameraSensorWrapper.update(imageRaster);
		markerSystemState.update(cameraSensorWrapper);
		// init 3D point list
		List<Point3> points3dlist = new ArrayList<>();

		List<Point> points2dlist = new ArrayList<Point>();
		for (int i = 0; i < ids.length; i++) {
			// process only if this marker has been detected
			if (markerSystemState.isExistMarker(ids[i])) {
				// read and add 2D points
				NyARIntPoint2d[] vertex2d = markerSystemState.getMarkerVertex2D(ids[i]);
				Point p = new Point(vertex2d[0].x, vertex2d[0].y);
				points2dlist.add(p);
				p = new Point(vertex2d[1].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[2].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[3].x, vertex2d[3].y);
				points2dlist.add(p);

				MatOfPoint mop = new MatOfPoint();
				mop.fromList(points2dlist);
				List<MatOfPoint> pts = new ArrayList<MatOfPoint>();
				pts.add(mop);
				// read and add corresponding 3D points
				points3dlist.addAll(markerConfig.create3dpointlist(patternmap.get(ids[i])));
			}

		}
		// load 2D and 3D points to Mats for solvePNP
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(points3dlist);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(points2dlist);

		// do not call solvePNP with empty intput data (no markers detected)
		if (points2dlist.size() == 0)
			return false;

		// uncomment these lines if using RANSAC-based pose estimation (more
		// shaking)
		Mat inliers = new Mat();

		Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, 300, 5, 16,
				inliers, Calib3d.CV_P3P);
		ARLoc.getLog().debug("Points detected: " + points2dlist.size() + " inliers: " + inliers.size());
		// avoid publish zero pose if localization failed
		if (inliers.rows() == 0)
			return false;

		return true;

	}

}

/**
 * Helper class to convert and OpenCV Mat containing a camera image to
 * NyARRGBRaster
 *
 */
class NyARImageHelper extends NyARRgbRaster {

	/**
	 * @param image
	 * @return
	 */
	public static INyARRgbRaster createFromMat(Mat image) {
		BufferedImage bimg;
		if (image != null) {
			int cols = image.cols();
			int rows = image.rows();
			int elemSize = (int) image.elemSize();
			byte[] data = new byte[cols * rows * elemSize];
			int type;
			image.get(0, 0, data);
			// we only support RGB
			type = BufferedImage.TYPE_3BYTE_BGR;
			// bgr to rgb
			byte b;
			for (int i = 0; i < data.length; i = i + 3) {
				b = data[i];
				data[i] = data[i + 2];
				data[i + 2] = b;
			}

			bimg = new BufferedImage(cols, rows, type);

			bimg.getRaster().setDataElements(0, 0, cols, rows, data);
		} else {
			bimg = null;
		}

		NyARImageHelper ra = null;

		int raster_type = NyARBufferType.BYTE1D_B8G8R8_24;
		try {
			ra = new NyARImageHelper(bimg.getWidth(), bimg.getHeight(), raster_type, false);
			ra._buf = ((DataBufferByte) (bimg.getRaster().getDataBuffer())).getData();
			ra._rgb_pixel_driver.switchRaster(ra);
		} catch (NyARException e) {
			e.printStackTrace();
		}

		return ra;

	}

	/**
	 * @param i_width
	 * @param i_height
	 * @param i_raster_type
	 * @param i_is_alloc
	 * @throws NyARException
	 */
	private NyARImageHelper(int i_width, int i_height, int i_raster_type, boolean i_is_alloc) throws NyARException {

		super(i_width, i_height, i_raster_type, i_is_alloc);
	}

}
