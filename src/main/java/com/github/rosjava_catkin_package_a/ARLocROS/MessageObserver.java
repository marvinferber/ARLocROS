package com.github.rosjava_catkin_package_a.ARLocROS;

import org.ros.internal.message.Message;

/**
 * @author Hoang Tung Dinh
 */
public interface MessageObserver<T extends Message> {
    void onNewMessage(T message);
}
