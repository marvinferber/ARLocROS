package com.github.rosjava_catkin_package_a.ARLocROS;

import com.google.auto.value.AutoValue;
import org.opencv.core.Point3;

/**
 * @author Hoang Tung Dinh
 */
@AutoValue
public abstract class Marker {

    Marker() {}

    abstract String patternFile();

    abstract Point3 upperleft();

    abstract Point3 upperright();

    abstract Point3 lowerright();

    abstract Point3 lowerleft();

    static Builder builder() {
        return new AutoValue_Marker.Builder();
    }

    @AutoValue.Builder
    abstract static class Builder {
        abstract Builder patternFile(String value);

        abstract Builder upperleft(Point3 value);

        abstract Builder upperright(Point3 value);

        abstract Builder lowerright(Point3 value);

        abstract Builder lowerleft(Point3 value);

        abstract Marker build();
    }
}