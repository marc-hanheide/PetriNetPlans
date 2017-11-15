import rospy
from abc import ABCMeta, abstractproperty
from AbstractCondition import AbstractCondition

class AbstractServiceCondition(AbstractCondition):
    __metaclass__ = ABCMeta

    def __init__(self):
        # create service proxy
        self.service_proxy = rospy.ServiceProxy(self._service_name, self._service_type)


    @abstractproperty
    def _service_name(self):
        raise NotImplementedError()

    @abstractproperty
    def _service_type(self):
        raise NotImplementedError()
