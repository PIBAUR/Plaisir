#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import threading

import rospy

class LogProcessThread(threading.Thread):
    def __init__(self, prefix, iterator):
        super(LogProcessThread, self).__init__()
        
        self.prefix = prefix
        self.iterator = iterator
        
    
    def run(self):
        print self.prefix + "log process started"
        for line in self.iterator:
            print self.prefix + line
        print self.prefix + "log process ended"
        


def execute():
    global processes
    
    processesChanged = False
    
    for robotId in processes.keys():
        robotIp = robotsBaseIp + robotId
        #print "ping " + robotIp
        pingResult = os.popen("ping " + robotIp + " -c 1").read()
        if pingResult.find("1 packets transmitted, 1 received") >= 0:
            #print "succeed"
            if processes[robotId] is None:
                cmd = ["/bin/bash", os.path.join(os.path.expanduser("~"), "catkin_ws/run/robot/launch_robots.sh"), robotId]
                
                print cmd
                process = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
                print "started /robot" + robotId + " process with PID " + str(process.pid)
                
                stdoutLogProcessThread = LogProcessThread("[INFO] [ /robot" + robotId + " ]: ", iter(process.stdout.readline, ""))
                stdoutLogProcessThread.start()
                stderrLogProcessThread = LogProcessThread("[ERROR] [ /robot" + robotId + " ]: ", iter(process.stderr.readline, ""))
                stderrLogProcessThread.start()
                
                processes[robotId] = (process, stdoutLogProcessThread, stderrLogProcessThread)
                
                processesChanged = True
            elif processes[robotId][0].poll() != None :
                #TODO: Check this
                """ 
                Add to check if process still alive
                May need review !!!
                """
                #process has terminated
                print "/robot" + robotId + " process PID  " + str(processes[robotId][0].pid) + " has terminated..."
                #processes[robotId][0].kill()
                processes[robotId] = None
                processesChanged = True
        else:
            #print "fault"
            if processes[robotId] is not None:
                print "kill /robot" + robotId + " process PID " + str(processes[robotId][0].pid)
                processes[robotId][0].kill()
                
                processes[robotId] = None
                
                processesChanged = True
    
    print "------------- RUNNING ROBOTS -------------------"
    for processKey, processValue in processes.items():
        if processValue is not None:
            print "--      /robot" + processKey + ": PID " + str(processValue[0].pid)
    print "------------------------------------------------"
    
    
if __name__ == '__main__':
    rospy.init_node("installation_manager_node", log_level = rospy.INFO)
    
    numRobots = rospy.get_param("num_robots", 7)
    robotsBaseIp = rospy.get_param("robots_base_ip", "192.168.1.2")
    
    processes = {}
    for i in range(numRobots):
        robotId = "0" * (2 - len(str(i))) + str(i)
        processes[robotId] = None
    
    firstTime = True
    while not rospy.is_shutdown():
        execute()
        rospy.sleep(5.)
        
        """
        if firstTime:
            firstTime = False
            
            process = subprocess.Popen(["python", os.path.expanduser("~") + "/catkin_ws/src/gui_execution_diagram/src/__init__.py", os.path.expanduser("~") + "/catkin_ws/params/default.dge", "--switch-to-multi-robots", "--start"], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
            
            stdoutLogProcessThread = LogProcessThread("[INFO] [ gui_execution_diagram ]: ", iter(process.stdout.readline, ""))
            stdoutLogProcessThread.start()
            stderrLogProcessThread = LogProcessThread("[ERROR] [ gui_execution_diagram ]: ", iter(process.stderr.readline, ""))
            stderrLogProcessThread.start()"""
    
    # kill for end
    for process in processes.values():
        if process is not None:
            process[0].kill()
