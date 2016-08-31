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

import it.unimi.dsi.fastutil.ints.Int2ObjectLinkedOpenHashMap;
import it.unimi.dsi.fastutil.ints.Int2ObjectMap;
import jp.nyatla.nyartoolkit.core.NyARCode;
import jp.nyatla.nyartoolkit.core.NyARException;
import jp.nyatla.nyartoolkit.core.param.NyARCameraDistortionFactorV2;
import jp.nyatla.nyartoolkit.core.param.NyARParam;
import jp.nyatla.nyartoolkit.core.param.NyARPerspectiveProjectionMatrix;
import jp.nyatla.nyartoolkit.core.raster.rgb.INyARRgbRaster;
import jp.nyatla.nyartoolkit.core.types.NyARIntPoint2d;
import jp.nyatla.nyartoolkit.core.types.NyARIntSize;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystem;
import jp.nyatla.nyartoolkit.markersystem.NyARMarkerSystemConfig;
import jp.nyatla.nyartoolkit.markersystem.NyARSensor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Static class that contains the pose computation from multiple AR marker
 * system using NyARToolkit Java library.
 * http://nyatla.jp/nyartoolkit/wp/?page_id=198
 */
public final class ComputePose {

	private final List<String> markerPatterns;
	private Map<Integer, String> patternmap;
	private final NyARIntSize i_screen_size;
	private final NyARPerspectiveProjectionMatrix i_projection_mat;
	private final NyARCameraDistortionFactorV2 i_dist_factor;
	private final NyARParam i_param;
	private NyARMarkerSystemConfig i_config;
	private NyARMarkerSystem markerSystemState;
	private NyARSensor cameraSensorWrapper;
	private int[] ids;
	private final MarkerConfig markerConfig;
	private final Mat cameraMatrix;
	private final MatOfDouble distCoeffs;
	private final Int2ObjectMap<NyARCode> arCodes = new Int2ObjectLinkedOpenHashMap<>();
	private final boolean visualization;
	

	private ComputePose(MarkerConfig markerConfig, Size size, Mat cameraMatrix, MatOfDouble
			distCoeffs, boolean visualization)
			throws NyARException, FileNotFoundException {
		this.visualization = visualization;
		this.markerConfig = markerConfig;
		this.cameraMatrix = cameraMatrix;
		this.distCoeffs = distCoeffs;
		// only load the whole configuration once
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
			arCodes.put(i, code);
			ids[i] = markerSystemState.addARMarker(code, 25, markerConfig.getMarkerSize());
			patternmap.put(ids[i], markerPatterns.get(i));
		}
	}

	public static ComputePose create(MarkerConfig markerConfig, Size size, Mat cameraMatrix,
			MatOfDouble distCoeffs, boolean visualization)
			throws NyARException, FileNotFoundException {
		return new ComputePose(markerConfig, size, cameraMatrix, distCoeffs, visualization);
	}

	public boolean computePose(Mat rvec, Mat tvec, Mat image2) throws NyARException, FileNotFoundException {
		// convert image to NyAR style for processing
		final INyARRgbRaster imageRaster = NyARImageHelper.createFromMat(image2);
		
		// create new marker system configuration
		i_config = new NyARMarkerSystemConfig(i_param);
		markerSystemState = new NyARMarkerSystem(i_config);
		// Create wrapper that passes cam pictures to marker system
		cameraSensorWrapper = new NyARSensor(i_screen_size);
		ids = new int[markerPatterns.size()];
		patternmap=new HashMap<>();
		for (int i = 0; i < markerPatterns.size(); i++) {
			// create marker description from pattern file and add to marker
			// system
			ids[i] = markerSystemState.addARMarker(arCodes.get(i), 25, markerConfig.getMarkerSize());
			patternmap.put(ids[i], markerPatterns.get(i));
		}
		
		cameraSensorWrapper.update(imageRaster);
		markerSystemState.update(cameraSensorWrapper);

		// init 3D point list
		final List<Point3> points3dlist = new ArrayList<>();
		final List<Point> points2dlist = new ArrayList<>();

		for (final int id : ids) {
			// process only if this marker has been detected
			if (markerSystemState.isExistMarker(id) && markerSystemState.getConfidence(id)>0.7) {
				// read and add 2D points
				final NyARIntPoint2d[] vertex2d = markerSystemState.getMarkerVertex2D(id);
				Point p = new Point(vertex2d[0].x, vertex2d[0].y);
				points2dlist.add(p);
				p = new Point(vertex2d[1].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[2].x, vertex2d[2].y);
				points2dlist.add(p);
				p = new Point(vertex2d[3].x, vertex2d[3].y);
				points2dlist.add(p);

				final MatOfPoint mop = new MatOfPoint();
				mop.fromList(points2dlist);
				final List<MatOfPoint> pts = new ArrayList<>();
				pts.add(mop);
				// read and add corresponding 3D points
				points3dlist.addAll(markerConfig.create3dpointlist(patternmap.get(id)));
				// draw red rectangle around detected marker
				Core.rectangle(image2, new Point(vertex2d[0].x, vertex2d[0].y), new Point(vertex2d[2].x, vertex2d[2].y),
						new Scalar(0, 0, 255));
			}

		}
		// load 2D and 3D points to Mats for solvePNP
		final MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(points3dlist);
		final MatOfPoint2f imagePoints = new MatOfPoint2f();
		imagePoints.fromList(points2dlist);

		if (visualization) {
			// show image with markers detected
			Imshow.show(image2);
		}

		// do not call solvePNP with empty intput data (no markers detected)
		if (points2dlist.size() == 0) {
			return false;
		}

		// uncomment these lines if using RANSAC-based pose estimation (more
		// shaking)
		Mat inliers = new Mat();

		Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, 300, 5, 16,
				inliers, Calib3d.CV_P3P);
		ARLoc.getLog().info("Points detected: " + points2dlist.size() + " inliers: " + inliers.size());
		// avoid publish zero pose if localization failed
		if (inliers.rows() == 0) {
			return false;
		}

		return true;
	}
}