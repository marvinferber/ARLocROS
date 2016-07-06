package com.github.rosjava_catkin_package_a.ARLocROS;

import geometry_msgs.TransformStamped;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Transform;
import rosjava_tf_example.Transformer;
import tf2_msgs.TFMessage;

import java.util.List;

/**
 * @author Hoang Tung Dinh
 */
public final class TransformationService implements MessageListener<TFMessage> {

    private final Transformer transformer;

    private TransformationService(Transformer transformer) {
        this.transformer = transformer;
    }

    public static TransformationService create(ConnectedNode connectedNode) {
        final Transformer transformer = Transformer.create();
        transformer.setPrefix(GraphName.of(connectedNode.getParameterTree().getString("~tf_prefix", "")));
        final TransformationService transformationService = new TransformationService(transformer);
        final Subscriber<TFMessage> tfSubscriber = connectedNode.newSubscriber(GraphName.of("tf"),
                tf2_msgs.TFMessage._TYPE);
        tfSubscriber.addMessageListener(transformationService);
        return transformationService;
    }

    @Override
    public void onNewMessage(TFMessage tfMessage) {
        for (TransformStamped transform : tfMessage.getTransforms()) {
            transformer.updateTransform(transform);
        }
    }

    public boolean canTransform(GraphName targetFrame, GraphName sourceFrame) {
        return transformer.canTransform(targetFrame, sourceFrame);
    }

    public Transform lookupTransform(GraphName targetFrame, GraphName sourceFrame) {
        return transformer.lookupTransform(targetFrame, sourceFrame);
    }
}
