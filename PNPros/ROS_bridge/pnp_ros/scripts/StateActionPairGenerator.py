import os
import rospy, rosbag
import multiprocessing
from AbstractTopicCondition import ConditionListener
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStartStateActionSaverResponse,\
                         PNPStopStateActionSaver, PNPStopStateActionSaverResponse
# TODO have a service message for allof them

class StateActionPairGenerator(ConditionListener):

    def __init__(self, action_manager, condition_manager):
        self._action_manager = action_manager
        self._condition_manager = condition_manager

        self._actions_history = {}
        self._conditions_history = {}
        self._saving_files = {}
        self._saving_bags = {}
        self._bags_lock = multiprocessing.Lock()

        # Initialize the state action saver services
        self._start_sa_service_provider = rospy.Service("start_state_action_saver",
            PNPStartStateActionSaver, self._start_state_action_saver_cb)
        self._stop_sa_service_provider = rospy.Service("stop_state_action_saver",
            PNPStopStateActionSaver, self._stop_state_action_saver_cb)
        self._check_running_service_provider = rospy.Service("running_state_action_saver",
            PNPStartStateActionSaver, self._check_running_cb)

        self._condition_manager.register_condition_listener(self)

    def _check_running_cb(self, req):
        goal = req.goal

        if goal in self._saving_files.keys():
            return PNPStartStateActionSaverResponse(1)
        else:
            return PNPStartStateActionSaverResponse(0)

    def _start_state_action_saver_cb(self, req):
        goal = req.goal

        # Start saving the action-state pairs
        if not goal in self._saving_files.keys():
            # create new file
            file_name = goal + "_" + str(rospy.Time.now().to_nsec()) + ".txt"
            filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)),\
                    "../demonstrations", file_name)
            file = open(filepath, "w")

            # insert file in dict
            self._saving_files.update({
                goal : file
            })
            self._actions_history.update({
                goal : []
            })
            self._conditions_history.update({
                goal : []
            })

            rospy.loginfo("Started recording demonstration in " + filepath)

        else:
            rospy.logwarn("Saver for " + goal + " already running")
            return PNPStartStateActionSaverResponse(0)

        # Start saving in the bag
        if not goal in self._saving_bags.keys():
            # create new bag
            file_name = goal + "_" + str(rospy.Time.now().to_nsec()) + ".bag"
            folder = '%s/catkin_ws/data/bags'  % os.path.expanduser("~")
            filepath = os.path.join(folder, file_name)
            bag = rosbag.Bag(filepath, "w")

            # insert file in dict
            with self._bags_lock:
                self._saving_bags.update({
                    goal : bag
                })

            rospy.loginfo("Started dumping demonstration in bag " + filepath)
        else:
            rospy.logwarn("Bag saver for " + goal + " already running")
            return PNPStartStateActionSaverResponse(0)

        return PNPStartStateActionSaverResponse(1)

    def _stop_state_action_saver_cb(self, req):
        goal = req.goal

        # Stop the state-action pair saver
        if goal in self._saving_files.keys():
            # Save state actions in all open files
            file = self._saving_files[goal]
            for timestep in range(len(self._conditions_history[goal])):
                for state_el in self._conditions_history[goal][timestep]:
                    file.write(state_el + " ")
                file.write("\t")
                for ex_action in self._actions_history[goal][timestep]:
                    file.write(ex_action + " ")
                file.write("\n")

            # close the file
            self._saving_files[goal].close()

            # remove the file from the dict
            del self._saving_files[goal]
            del self._conditions_history[goal]
            del self._actions_history[goal]

            rospy.loginfo("Closed file " + goal)
        else:
            rospy.logwarn("No running saver in file " + goal + " found")
            return PNPStopStateActionSaverResponse(0)

        # Stop the bag saving
        if goal in self._saving_bags.keys():
            with self._bags_lock:
                self._saving_bags[goal].close()
                del self._saving_bags[goal]
                rospy.loginfo("Closed bag for " + goal)
        else:
            rospy.logwarn("No running bag saver for " + goal + " found")
            return PNPStopStateActionSaverResponse(0)

        return PNPStopStateActionSaverResponse(1)

    def receive_update(self, condition_instance):
        condition_name = condition_instance.get_name()
        condition_value = condition_instance.get_value()

        # Get the current conditions
        current_state = self._condition_manager.get_conditions_dump()

        # Get executable actions
        action_list = self._action_manager.get_actions()

        # Check the actions that reached the goal after this change
        action_candidates = []
        if condition_value.lower() != "none":
            for action in action_list:
                #print action, "goal reached?"
                if self._action_manager.is_goal_reached(action, condition_value.split("_")):
                    action_candidates.append(action + "_".join(["", condition_value]))

        # Save in state-action pairs file
        for goal_name in self._saving_files.keys():
            # Save the condition
            self._conditions_history[goal_name].append(current_state)
            # spread the action to the previous timesteps without actions
            executed_actions = action_candidates[:]
            for timestep in range(len(self._actions_history[goal_name])-1, -1, -1):
                if len(self._actions_history[goal_name][timestep]) == 0:
                    self._actions_history[goal_name][timestep] = executed_actions
                else:
                    break

            # Add empty container of actions for the current step
            self._actions_history[goal_name].append([])

            # sometimes this assert is false, maybe caused by parallel access at the class variables
            #assert(len(self._conditions_history[goal_name])==len(self._actions_history[goal_name]))

        # Save in the bag
        topic_message = condition_instance.get_data() # the actual topic message
        topic_name = condition_instance._topic_name # the type of topic message
        with self._bags_lock:
            for bag in self._saving_bags.values():
                #print topic_name, topic_message
                bag.write(topic_name, topic_message)
