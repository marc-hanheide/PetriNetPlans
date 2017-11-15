import rospy
from abc import ABCMeta, abstractmethod

class AbstractCondition():
    __metaclass__ = ABCMeta

    @abstractmethod
    def evaluate(self, params):
        raise NotImplementedError()
