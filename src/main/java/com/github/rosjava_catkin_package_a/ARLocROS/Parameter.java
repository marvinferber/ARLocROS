package com.github.rosjava_catkin_package_a.ARLocROS;

import com.google.auto.value.AutoValue;
import org.ros.node.parameter.ParameterTree;

/**
 * @author Hoang Tung Dinh
 */
@AutoValue
public abstract class Parameter {

    protected Parameter() {}

    public abstract String patternDirectory();

    public abstract String markerFrameName();

    public abstract String cameraFrameName();

    public abstract String cameraImageTopic();

    public abstract String cameraInfoTopic();

    public abstract String markerConfigFile();

    public abstract boolean badPoseReject();

    public static Parameter createFromParameterTree(ParameterTree parameterTree) {
        return builder().patternDirectory(parameterTree.getString("/ARLocROS/pattern_dir"))
                .markerConfigFile(parameterTree.getString("/ARLocROS/marker_config_file"))
                .markerFrameName(parameterTree.getString("/ARLocROS/marker_frame_name"))
                .cameraFrameName(parameterTree.getString("/ARLocROS/camera_frame_name"))
                .cameraImageTopic(parameterTree.getString("/ARLocROS/camera_image_topic"))
                .cameraInfoTopic(parameterTree.getString("/ARLocROS/camera_info_topic"))
                .badPoseReject(parameterTree.getBoolean("/ARLocROS/bad_pose_reject"))
                .build();
    }

    public static Builder builder() {
        return new AutoValue_Parameter.Builder();
    }

    @AutoValue.Builder
    public abstract static class Builder {
        public abstract Builder patternDirectory(String value);

        public abstract Builder markerFrameName(String value);

        public abstract Builder cameraFrameName(String value);

        public abstract Builder cameraImageTopic(String value);

        public abstract Builder cameraInfoTopic(String value);

        public abstract Builder markerConfigFile(String value);

        public abstract Builder badPoseReject(boolean value);

        public abstract Parameter build();
    }
}
