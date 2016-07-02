package com.github.rosjava_catkin_package_a.ARLocROS;

import org.opencv.core.Point3;

/**
 * @author Hoang Tung Dinh
 */
public class Marker {

    public Marker(String patternFile) {
        this.patternFile = patternFile;
    }

    String patternFile;
    Point3 upperleft;
    Point3 upperright;
    Point3 lowerright;
    Point3 lowerleft;
}