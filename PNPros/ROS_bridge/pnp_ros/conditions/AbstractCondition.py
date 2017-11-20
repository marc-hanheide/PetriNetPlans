import rospy
from abc import ABCMeta, abstractmethod

class AbstractCondition():
    __metaclass__ = ABCMeta

    def __init__(self):
        self._updates_listeners = []

    @abstractmethod
    def evaluate(self, params):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self):
        raise NotImplementedError()

    def register_updates_listener(self, obj):
        if issubclass(obj.__class__, ConditionListener):
            self._updates_listeners.append(obj)
        else:
            rospy.logwarn("Object " + str(obj.__class__) + " is not a ConditionListener subclass, cannot be registered as listener")

    def _condition_name(self):
        return self.__class__.__name__

class ConditionListener():
    __metaclass__ = ABCMeta

    @abstractmethod
    def receive_update(self, condition_name, condition_value):
        raise NotImplementedError()
