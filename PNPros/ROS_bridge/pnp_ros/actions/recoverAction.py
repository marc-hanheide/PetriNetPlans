import rospy
import subprocess
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Twist
from pnp_msgs.msg import ActionFailure
from AbstractAction import AbstractAction

class recoverAction(AbstractAction):

    def _start_action(self):

        # stop the robot
        pub = rospy.Publisher("cmd_vel", Twist, latch=True, queue_size=10)
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        pub.publish(twist)

        # Ask for confirmation by human
        window = tk.Tk()
        self.confirmed = None

        def confirm_y():
            self.confirmed = True
            window.destroy()

        def confirm_n():
            self.confirmed = False
            window.destroy()

        label = ttk.Label(window, text="Was it actually a dangerous situation?")
        label.pack()
        by = ttk.Button(window, text="Yes", command = confirm_y)
        bn = ttk.Button(window, text="No", command = confirm_n)
        by.pack()
        bn.pack()
        window.mainloop()

        print "confirmed: ", self.confirmed

        if self.confirmed:
            ## apply recovery action

            pass
        else:
            ## signal wrong failure detection
            pub = rospy.Publisher("failure_signal", ActionFailure, queue_size=10, latch=True)
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.cause = "falsepositive"
            pub.publish(msg)

        # finished action
        self.params[len(self.params):] = ["done"]



    def _stop_action(self):

        pass

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        return False
