import os
import glob
import rospy
import rosbag
import inspect
import threading

from pnp_msgs.srv import PNPStartConditionsDump, PNPStopConditionsDump
from AbstractCondition import AbstractCondition, AbstractConditionListener
from importlib import import_module

class ConditionManager(AbstractConditionListener):

    _condition_instances = {}
    _bags = {}

    def __init__(self):

        # Initialize all the classes in current folder which implement AbstractCondition
        for file in glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "*.py")):
            name = os.path.splitext(os.path.basename(file))[0]
            try:
                condition_class = getattr(import_module(name), name)
            except (ImportError, AttributeError):
                continue
            else:
                if issubclass(condition_class, AbstractCondition) and not inspect.isabstract(condition_class):
                    # Instanciate the condition
                    condition_instance = condition_class()

                    self._condition_instances.update({
                        name : condition_instance
                    })

                    rospy.loginfo("Initialized condition " + name)
                else:
                    rospy.logwarn("Class " + name + " does not inherit from AbstractCondition")

        # Initialize the condition dumping services
        self.start_dump_service_provider = rospy.Service("start_conditions_dump", PNPStartConditionsDump, self._start_conditions_dump_cb)
        self.stop_dump_service_provider = rospy.Service("stop_conditions_dump", PNPStopConditionsDump, self._stop_conditions_dump_cb)

    def __del__(self):
        if self.start_dump_service_provider:
            self.start_dump_service_provider.shutdown()
        if self.stop_dump_service_provider:
            self.stop_dump_service_provider.shutdown()

    def evaluate(self, condition_name, params):
        try:
            return self._condition_instances[condition_name].evaluate(params)
        except KeyError:
            rospy.logwarn("Condition " + condition_name + " not implemented")
            # return true when the condition is not implemented, to avoid loops..
            return True

    def _start_conditions_dump_cb(self, req):
        bag_name = req.bag_name

        if not bag_name in self._bags.keys():
            # create new bag
            bag = rosbag.Bag(bag_name, "w")

            # insert bag in dict
            self._bags.update({
                bag_name : bag
            })

            return True
        else:
            rospy.logwarn("Dumping in bag " + bag_name + " already running")
            return False

    def _stop_conditions_dump_cb(self, req):
        bag_name = req.bag_name

        if bag_name in self._bags.keys():
            # close the bag
            self._bag[bag_name].close()

            # remove the bag from the dict
            del self._bag[bag_name]

            rospy.loginfo("Closed bag " + bag_name)
            return True
        else:
            rospy.logwarn("No running dumping in bag " + bag_name + " found")
            return False

    def receive_update(self, condition_name, condition_value):
        # save the new value in all the running bags
        for bag in self._bags.values():
            bag.write(condition_name, condition_value)
