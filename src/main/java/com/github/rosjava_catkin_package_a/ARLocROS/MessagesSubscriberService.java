package com.github.rosjava_catkin_package_a.ARLocROS;

import com.google.common.base.Optional;
import com.google.common.collect.EvictingQueue;
import com.google.common.collect.Queues;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * @author mhct
 */
public class MessagesSubscriberService<T extends Message> {

    private static final Logger logger = LoggerFactory.getLogger(MessagesSubscriberService.class);

    private final MessagesListener<T> messagesListener;

    private static final int DEFAULT_MESSAGE_QUEUE_SIZE = 1;

    protected MessagesSubscriberService(Subscriber<T> subscriber, int maxMessageQueueSize) {
        checkArgument(maxMessageQueueSize >= 1,
                String.format("Queue size must be at least 1, but it is %d.", maxMessageQueueSize));
        this.messagesListener = MessagesListener.create(maxMessageQueueSize);
        subscriber.addMessageListener(messagesListener);
    }

    public static <Type extends Message> MessagesSubscriberService<Type> create(Subscriber<Type> subscriber) {
        return new MessagesSubscriberService<>(subscriber, DEFAULT_MESSAGE_QUEUE_SIZE);
    }

    public static <Type extends Message> MessagesSubscriberService<Type> create(Subscriber<Type> subscriber,
            int maxMessageQueueSize) {
        return new MessagesSubscriberService<>(subscriber, maxMessageQueueSize);
    }

    public Optional<T> getMostRecentMessage() {
        return messagesListener.getMostRecentMessage();
    }

    public Queue<T> getMessageQueue() {
        return messagesListener.getMessageQueue();
    }

    public void registerMessageObserver(MessageObserver<T> messageObserver) {
        messagesListener.registerMessageObserver(messageObserver);
    }

    public void removeMessageObserver(MessageObserver<T> messageObserver) {
        messagesListener.removeMessageObserver(messageObserver);
    }

    private static final class MessagesListener<K extends Message> implements MessageListener<K> {
        private final Collection<MessageObserver<K>> messageObservers;
        private final Queue<K> messageQueue;
        @Nullable private K mostRecentMessage;

        private MessagesListener(int maxQueueSize) {
            messageQueue = Queues.synchronizedQueue(EvictingQueue.<K>create(maxQueueSize));
            messageObservers = new ArrayList<>();
        }

        public static <Type extends Message> MessagesListener<Type> create(int maxQueueSize) {
            return new MessagesListener<>(maxQueueSize);
        }

        @Override
        public void onNewMessage(K newMessage) {
            logger.trace("{} {}", System.nanoTime() / 1000000000.0, newMessage.toRawMessage().getType());
            messageQueue.add(newMessage);
            mostRecentMessage = newMessage;
            notifyMessageObservers(newMessage);
        }

        private void notifyMessageObservers(K t) {
            for (final MessageObserver<K> msgObs : messageObservers) {
                msgObs.onNewMessage(t);
            }
        }

        public void registerMessageObserver(MessageObserver<K> messageObserver) {
            messageObservers.add(messageObserver);
        }

        public void removeMessageObserver(MessageObserver<K> messageObserver) {
            messageObservers.remove(messageObserver);
        }

        Optional<K> getMostRecentMessage() {
            if (mostRecentMessage == null) {
                return Optional.<K>absent();
            } else {
                return Optional.<K>of(mostRecentMessage);
            }
        }

        Queue<K> getMessageQueue() {
            return new LinkedList<>(messageQueue);
        }
    }
}
