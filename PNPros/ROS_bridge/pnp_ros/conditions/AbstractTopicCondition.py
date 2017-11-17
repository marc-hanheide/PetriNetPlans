import rospy
from abc import ABCMeta, abstractproperty, abstractmethod
from AbstractCondition import AbstractCondition

class AbstractTopicCondition(AbstractCondition):
    __metaclass__ = ABCMeta

    def __init__(self):
        # subscribe to the topic with a callback
        rospy.Subscriber(self._topic_name, self._topic_type, self._callback)

        # last_data will be None until the subscribed topic will return some data
        self.last_value = None

    def _callback(self, data):
        self.last_value = self._get_value_from_data(data)

        # update all the listeners
        for listener in self._updates_listeners:
            listener.receive_update(self.__name__, self.last_value)

    @abstractmethod
    def _get_value_from_data(self, data):
        raise NotImplementedError()

    @abstractproperty
    def _topic_name(self):
        raise NotImplementedError()

    @abstractproperty
    def _topic_type(self):
        raise NotImplementedError()
