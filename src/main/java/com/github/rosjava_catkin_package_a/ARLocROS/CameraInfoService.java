package com.github.rosjava_catkin_package_a.ARLocROS;

import com.google.common.base.Optional;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import sensor_msgs.CameraInfo;

import javax.annotation.Nullable;

/**
 * This class is designed with the assumption that camera info never changes during the runtime. Therefore, it will
 * only listen to camera info once.
 * // TODO test this class
 *
 * @author Hoang Tung Dinh
 */
public final class CameraInfoService {
    private static final Logger logger = LoggerFactory.getLogger(CameraInfoService.class);

    private final Subscriber<CameraInfo> cameraInfoSubscriber;
    @Nullable private CameraParams cameraParams;

    private CameraInfoService(Subscriber<CameraInfo> cameraInfoSubscriber) {
        this.cameraInfoSubscriber = cameraInfoSubscriber;
        this.cameraInfoSubscriber.addMessageListener(new CameraInfoListener());
    }

    public static CameraInfoService create(Subscriber<CameraInfo> cameraInfoSubscriber) {
        return new CameraInfoService(cameraInfoSubscriber);
    }

    public Optional<CameraParams> getCameraParams() {
        if (cameraParams == null) {
            return Optional.absent();
        } else {
            return Optional.of(cameraParams);
        }
    }

    private final class CameraInfoListener implements MessageListener<CameraInfo> {

        private CameraInfoListener() {}

        @Override
        public void onNewMessage(CameraInfo cameraInfo) {
            if (cameraParams == null) {
                cameraParams = CameraParams.builder()
                        .fx(cameraInfo.getK()[0])
                        .fy(cameraInfo.getK()[4])
                        .cx(cameraInfo.getK()[2])
                        .cy(cameraInfo.getK()[5])
                        .k1(cameraInfo.getD()[0])
                        .k2(cameraInfo.getD()[1])
                        .p1(cameraInfo.getD()[2])
                        .p2(cameraInfo.getD()[3])
                        .width(cameraInfo.getWidth())
                        .height(cameraInfo.getHeight())
                        .frame_id(cameraInfo.getHeader().getFrameId())
                        .build();

                cameraInfoSubscriber.shutdown();

                logger.info("Setting up camera parameters");
            }
        }
    }
}
