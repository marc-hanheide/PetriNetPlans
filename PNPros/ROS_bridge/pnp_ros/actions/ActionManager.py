import rospy
from importlib import import_module
from AbstractAction import AbstractAction
from pnp_msgs.msg import PNPResult, PNPGoal

class ActionManager():


    _action_instances = {}

    def __init__(self):
        # start interrupted_goal topic
        self._interrupted_goal_publisher = rospy.Publisher('interrupted_goal', PNPGoal, queue_size=10, latch=True)


    @staticmethod
    def _find_action_implementation(action_name):
        try:
            action_class = getattr(import_module(action_name), action_name)
        except (ImportError, AttributeError):
            rospy.logwarn("action " + action_name + " not implemented")
        else:
            if issubclass(action_class, AbstractAction):
                return action_class
            else:
                rospy.logwarn("class " + action_class + " must inherit from AbstractAction")

    def start_action(self, goalhandler):
        goal = goalhandler.get_goal()
        print "Starting " + goal.name + " " + goal.params

        # search for an implementation of the action
        action = self._find_action_implementation(goal.name)

        if action:
            # accept the goal
            goalhandler.set_accepted()

            ## check that the action goal is not already reached
            if not self.is_goal_reached(goal.name, goal.params.split("_")):
                # Instantiate the action
                action_instance = action(goalhandler, goal.params.split("_"))

                # add action instance to the dict
                self._action_instances.update({
                    goal.id : action_instance
                })

                # start the action
                self._action_instances[goal.id].start_action()
            else:
                # send the result
                result = PNPResult()
                result.result = 'OK'
                goalhandler.set_succeeded(result, 'OK')

    def interrupt_action(self, goalhandler):
        ''' Action interrupted before it finished the execution '''
        goal = goalhandler.get_goal()
        print "Interrupting " + goal.name + " " + goal.params

        # accept the goal
        goalhandler.set_accepted()

        # stop the action
        if goal.id in self._action_instances.keys():
            self._action_instances[goal.id].interrupt_action()

            # publish interrupted goal
            self._interrupted_goal_publisher.publish(goal)

            # remove instance
            del self._action_instances[goal.id]


    def end_action(self, goalhandler):
        ''' Action ended its execution (this is called after the action is already finished)'''
        goal = goalhandler.get_goal()
        print "Ending " + goal.name + " " + goal.params

        # accept the goal
        goalhandler.set_accepted()

        # end the action
        if goal.id in self._action_instances.keys():

            # remove instance
            del self._action_instances[goal.id]


    @staticmethod
    def is_goal_reached(action_name, parameters):
        # search for an implementation of the action
        action = ActionManager._find_action_implementation(action_name)


        if action:
            # Instantiate the action
            return action.is_goal_reached(parameters)
        else:
            return False
