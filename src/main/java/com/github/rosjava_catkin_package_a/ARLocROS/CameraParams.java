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

import com.google.auto.value.AutoValue;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Size;

/**
 * Helper class to setup camera parameters as OpenCV Mat.
 */
@AutoValue
public abstract class CameraParams {

    CameraParams() {}

    abstract double fx();

    abstract double fy();

    abstract double cx();

    abstract double cy();

    abstract double k1();

    abstract double k2();

    abstract double p1();

    abstract double p2();

    abstract int width();

    abstract int height();

    abstract String frame_id();

    public static Mat getCameraMatrix(CameraParams cameraParams) {
        final Mat cameraMatrix = new Mat(new Size(3, 3), CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParams.fx());
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cameraParams.cx());
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, cameraParams.fy());
        cameraMatrix.put(1, 2, cameraParams.cy());
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);

        return cameraMatrix;
    }

    public static MatOfDouble getDistCoeffs(CameraParams cameraParams) {
        final MatOfDouble distCoeffs = new MatOfDouble(new Mat(4, 1, CvType.CV_64FC1));
        distCoeffs.put(0, 0, cameraParams.k1());
        distCoeffs.put(1, 0, cameraParams.k2());
        distCoeffs.put(2, 0, cameraParams.p1());
        distCoeffs.put(3, 0, cameraParams.p2());
        return distCoeffs;
    }

    public static Builder builder() {
        return new AutoValue_CameraParams.Builder();
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder fx(double value);

        public abstract Builder fy(double value);

        public abstract Builder cx(double value);

        public abstract Builder cy(double value);

        public abstract Builder k1(double value);

        public abstract Builder k2(double value);

        public abstract Builder p1(double value);

        public abstract Builder p2(double value);

        public abstract Builder width(int value);

        public abstract Builder height(int value);

        public abstract Builder frame_id(String value);

        public abstract CameraParams build();
    }

}
