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
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

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

    /**
     * @param cameraMatrix
     * @param distCoeffs2
     * @param camps
     */
    public static void getCameraParamas(Mat cameraMatrix, MatOfDouble distCoeffs2, CameraParams camps) {
        cameraMatrix.put(0, 0, camps.fx());
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, camps.cx());
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, camps.fy());
        cameraMatrix.put(1, 2, camps.cy());
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
        distCoeffs2.put(0, 0, camps.k1());
        distCoeffs2.put(1, 0, camps.k2());
        distCoeffs2.put(2, 0, camps.p1());
        distCoeffs2.put(3, 0, camps.p2());
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
