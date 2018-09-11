import rospy

TOPIC_PLANTOEXEC = "planToExec"
TOPIC_PNPACTIONCMD = "PNPActionCmd"
SRV_PNPCONDITIONEVAL = "PNPConditionEval"
SRV_PNPCONDITIONVALUE = "PNPConditionValue"
PARAM_PNPACTIONSTATUS = "/PNPActionStatus/"
PARAM_PNPCONDITIONBUFFER = "PNPconditionsBuffer/"

ACTION_STARTED = "started"
ACTION_RUNNING = "running"
ACTION_INTERRUPT = "interrupt"
ACTION_SUCCESS = "success"
ACTION_NOT_IMPLEMENTED = "not_implemented"

# PNPPLANFOLDER = "pnp_ros/plan_folder"


def get_robot_key(name):
    key = name
    if rospy.has_param('robotname'):
        key = "/"+rospy.get_param('robotname')+"/"+key
    return key
