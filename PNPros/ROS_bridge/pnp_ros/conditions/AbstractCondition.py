import rospy
from abc import ABCMeta, abstractmethod

class AbstractCondition():
    __metaclass__ = ABCMeta

    _updates_listeners = []

    @abstractmethod
    def evaluate(self, params):
        raise NotImplementedError()

    def register_updates_listener(self, obj):
        if issubclass(obj, AbstractConditionListener):
            self._updates_listeners.append(obj)
        else:
            rospy.logwarn("Object " + obj + " is not a AbstractConditionListener subclass, cannot be registered as listener")

class AbstractConditionListener():
    __metaclass__ = ABCMeta

    @abstractmethod
    def receive_update(self, condition_name, condition_value):
        raise NotImplementedError()
