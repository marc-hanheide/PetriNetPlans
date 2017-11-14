import qi
import argparse
import sys
import time
import threading
import os

G_actionThread_exec = {}  # action thread execution functions
G_actionThreads = {}  # action threads functions

cakey = "PNP/CurrentAction"

acb = None # action callback

def action_cb(value):
    global  G_actionThreads, G_actionThread_exec, G_memory_service, G_session
    v = value.split()
    print "action_cb value ",value
    if (v[0]=='start'):
        actionName = v[1]
        params=""
        if (len(v)>2):
            params=v[2]
        
        if (actionName in G_actionThread_exec):
            G_actionThreads[actionName] = threading.Thread(target = G_actionThread_exec[v[1]], args=(params,))
            G_actionThreads[actionName].mem_serv = G_memory_service
            G_actionThreads[actionName].session = G_session
            G_actionThreads[actionName].start()
            G_memory_service.raiseEvent(cakey,actionName+"_"+params)
        else:
            print "ERROR: Action ",v[1]," not found !!!"
    elif (v[0]=='end' or v[0]=='stop' or v[0]=='interrupt'):
        try:
            actionName = v[1]
            G_actionThreads[actionName].do_run = False  # execution thread associated to actionName
            G_memory_service.raiseEvent(cakey,"")
            # print "DEBUG: action ",actionName," ended.  Thread ",G_actionThreads[actionName]
        except:
            print "ERROR: Action ",v[1]," not started !!!"


def initApp(actionName):
    
	parser = argparse.ArgumentParser()
	parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
	parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
	args = parser.parse_args()
	pip = args.pip
	pport = args.pport

	#Starting application
	try:
		connection_url = "tcp://" + pip + ":" + str(pport)
		print "Connecting to ",	connection_url
		app = qi.Application(["Action_"+actionName, "--qi-url=" + connection_url ])
	except RuntimeError:
		print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
		sys.exit(1)

	app.start()
	return app


def init(session, actionName, actionThread_exec):
    global G_actionThread_exec, G_memory_service, G_session, acb
    G_actionThread_exec[actionName] = actionThread_exec # execution thread function associated to actionName
    G_session = session

    G_memory_service  = session.service("ALMemory")

    #subscribe to PNP action event
    if (acb==None):
        acb = G_memory_service.subscriber("PNP_action")
        acb.signal.connect(action_cb)

    G_memory_service.declareEvent("PNP_action_result_"+actionName);
    G_memory_service.declareEvent("PNP/CurrentAction");


    print "Naoqi Action server "+actionName+" running..."





