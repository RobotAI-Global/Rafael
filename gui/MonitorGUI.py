# -*- coding: utf-8 -*-
"""

@author: avita

RobotAI : Robot Manager GUI
Usage :


Install:
    Zion -> env :  D:/RobotAI/Design/env/inspect6d/python.exe : matplotlib 3.0.3

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101    11.06.24 UD     Created
-----------------------------

"""
gui_version   = '0101'

import os
#import tkinter as tk
#import threading
#import json
import logging as log
import time   # just to measure switch time
#import logging
import webbrowser

#base_path     = os.path.abspath(".")
#base_path      = r'D:\RobotAI\Design\apps\PickManager\gui\logo.ico' #os.path.dirname(os.path.realpath(__file__))
iconFilePath  =  '.\\gui\\logo.ico' #os.path.join(base_path, '\logo.ico')
#base_path      = os.path.abspath(".")
#iconFilePath  =  r'C:\robotai\SW\RobotManager\gui\logo.ico' #os.path.join(base_path, '.\\gui\\logo.ico')

#import os
import sys
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\robot')
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\vision')

#import os

sys.path.insert(1, r'.\\robot')
#sys.path.insert(1, r'.\\vision')

#%%
import tkinter as tk
import tkinter.ttk as ttk
#import customtkinter as ctk
#from tkinter import *

#from MonitorMessage import Message, MESSAGE_TYPE
#import atexit

# def exit_handler():
#     print('My application is ending!')
#     closeSerial()
# atexit.register(exit_handler)

# global variables for this module
ledAstatus = 0
ledBstatus = 0
servoPos = 10
stopThread = False

#import matplotlib
#matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg , NavigationToolbar2Tk 
from matplotlib.figure import Figure
from matplotlib.backend_bases import key_press_handler
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread
style.use("dark_background")

#import matplotlib as mpl
#mpl.rcParams['toolbar'] = 'None' 


#try:
from gui.ConfigManager import ConfigManager
from gui.DisplayManager  import DisplayManager
from robot.RobotAPI import RobotAPI as RobotManager
from disc.ControllerHHC import IOController
from gui.TkinkerJson  import JsonEditor
from gui.PoseGUI import PoseGUI

#except Exception as e:
#    from ConfigManager import ConfigManager
#    from DisplayManager  import DisplayManager
#    from RobotAPI import RobotAPI as RobotManager    
#    # debug
#    print(e)
#    print('Load them manually and then switch directory')
#    #from MsgJsonSimple import MsgPoseData
#

#ion() # turn interactive mode on
    

#import tkinter as tk

#def FromUnsignedToSigned16(n):
#    return n - 0x10000 if n & 0x8000 else n
#def FromSignedToUnsigned16(n):
#    return 0x10000 + n if n < 0 else n



AXIS_VIEW_OPTIONS             = ["45-45","Left","Top"] #et
SAFETY_MARGIN_Z               = 50  # mm
TX_STEP_SIZE                  = 0.0400 # metr
SCAN_POINTS                   = [[-500,-200, 400, 94, 0, -82],[-500,0,400, 94, 0, -82],[-500,0,600, 94, 0, -82],[-500,-220,600, 94, 0, -82],[-665.01 ,-251.23 , 648.96 , 94, 0, -82]]
HOME_POSE                     = [-500.0, 38.0, 430.0, 94.3088530785335, 0.6875493541569879, -82.21944360127314]
        
WORK_POINTS  = [[-0.520,-0.200, 0.400, -1.64,0, 1.7],   [-0.750,-0.200, 0.400, -1.64,0, 1.7],
                [-0.500, 0,     0.400, -1.64,0, 1.7],   [-0.750, 0,     0.400, -1.64,0, 1.7],
                [-0.500, 0,     0.600, -1.64,0, 1.7],   [-0.650, 0,     0.600, -1.64,0, 1.7],
                [-0.500,-0.220, 0.600, -1.64,0, 1.7],   [-0.750,-0.220, 0.600, -1.64,0, 1.7]]


#    cfg         = module_dict['cfg']
#    # prj         = module_dict['prj'] # load session
#    # cam         = module_dict['cam']
#    # obj         = module_dict['obj']
#    # rob         = module_dict['rob']
#    # lbl         = module_dict['lbl']

class MonitorGUI:

    def __init__(self, win, cfg = None):

        self.win        = win
        self.cfg        = ConfigManager()
        self.debugOn    = False
        self.ts         = None # task handle
        
        # robot comm
        #self.com        = None #MonitorComm(cfg)        
        
        #self.ip_robot  = "192.168.1.16"
        #self.port_robot = 5033
        #self.rbm        = None  # robot manager
        self.rbm        = RobotManager(self) #ip = self.ip_robot, port = self.port_robot) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
        self.ioc        = IOController(self)

        self.robot_speed = 100  # some strnage numbers
        self.robot_pose  = np.zeros((1,6))
        #self.home_pose   = np.array([-86.2, -225.6, 237.6, -177.0, 2.29, 138.6]) # chess
        self.home_pose   = np.array(HOME_POSE)
        
        # receive message
        self.msgRecv   = {'Id': 0, 'Data' : 0}
        
        # host
        self.ip_vision         = '127.0.0.1'    
        #self.ip_vision         = '192.168.1.130' # Uri comp
        self.port_vision       = 5555
        self.vis        = None #VisionManager(self.cfg)        
        self.sim        = None #MonitorComm(cfg)        
        self.sim_ts     = None  # simulator task handle
        self.object_pose  = np.zeros((1,6))
        
##        # robot
#        self.ip     = '192.168.0.10'
#        self.port   = 49154
        
        
#        if self.cfg is None:
#            self.cfg = ConfigManager()
            
        self.dsp       = DisplayManager(self.cfg)


        # ugly but simple
        self.sliderCount = 1
        
    ## ------------------------------------  
    # # -- TASK COMM ---
    # def commServer(self):
    #    # start
    #    self.tprint('Waiting for Robot connection ...')
       
    #    # maybe already running
    #    if self.com is None:
    #        # runs Robot server and Multi Object Detection
    #        self.com    = hostServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
    #        self.com.start()
    #    elif self.com.is_alive():   
    #        self.tprint('Server is alive')
       
    # def commClient(self):
    #    # start
    #    self.tprint('Simulating Robot connection ...')
       
    #    # maybe already running
    #    if self.sim is None:
    #        # runs Robot server and Multi Object Detection
    #        self.sim    = hostClientThread(host = self.ip, port = self.port,  debug=self.debugOn)
    #    #elif  not self.com.isAlive():      
    #    self.sim.start()        
       
       

    # def commDisConnect(self):
    #    #
    #    self.tprint('Trying to stop Robot connection ...')
    #    self.com.stop()
    
    ## -----------------------------    
    # -- Session Control ---
    ## -----------------------------    
        
    def getVersion(self):
        # start
        self.tprint('Reading version  ...')
        
        # maybe already running
        self.tprint('App version : %s' %str(gui_version))
        
    def orderSelect(self):
        "edit parameter file"
        
        self.cfg.CreatePickOrderFileJson()
        fname = self.cfg.GetPickOrderFileName()
        webbrowser.open(fname) 
        self.tprint(fname)

    def sessionSelect(self):
        "edit session parameter file"
         
        self.cfg.CreateSessionFile()
        fname = self.cfg.GetSessionFileName()
        #webbrowser.open(fname)
        JsonEditor(fname)
        self.tprint(fname)         
                
        

        

    ## -----------------------------    
    # -- Robot Control ---
    ## -----------------------------    
        
    def robotConnect(self):
        # start
        self.tprint('Starting Robot connection ...')
        
        # maybe already running
        if self.rbm is None:
            # runs Robot server and Multi Object Detection
            self.tprint('Bad Robot Init  ... Abort') 
            return
            #self.rbm    = RobotManager(ip = self.ip_robot, port = self.port_robot, block_com=False, config=self.cfg) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
            #self.rbm.instanceSocket()
            #self.rbm.start()
        elif self.rbm.Connected():   
            self.tprint('Robot is alive')
        else:
            self.rbm.RobotData()
            self.tprint('Robot is restarted')
            
        #self.rbm.setPowerOn()
        #self.rbm.setReleaseBrake()

    def robotStop(self):
        # start
        self.tprint('Stop Robot  ...')
        
        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return
        
        self.rbm.Stop()

    def robotCommState(self):
        # comm problems?
        
        comm_state = self.rbm.GetRobotStatus()
        self.tprint("Robot status: %s" %str(comm_state))

    def robotStatus(self):
        # start
        self.tprint('Getting Robot status ...')

        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return
        
        stat = self.rbm.GetRobotStatus()
        self.tprint(str(stat))


    def robotDisConnect(self):
        # start
        self.tprint('Disconnect from Robot ...')
        
        # maybe already running
        if self.rbm is None:
            # runs 
            pass
        elif self.rbm.Connected():  
            self.rbm.Stop()
            self.rbm.RobotPower('off')            
            #self.rbm.stop()
        else:
            self.rbm.RobotPower('off')

        self.tprint('Robot is stopped')  
        
    def robotSetRobotSpeed(self):
        # set speed 1-100
        speedVal = np.maximum(10,np.minimum(200,self.sliderCount))        
        self.robot_speed = speedVal
        
        self.tprint('Robot speed set : %s' %str(speedVal)) 
        
    def robotGetGripperPose(self):
        # read pose
        self.tprint('Robot read pose ... ') 
        
        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return [0]*6
        
        res = self.rbm.GetPosition() #getTool_xyzrxryrz()
        robotPose = res[1]
        
        robotPose[0] = robotPose[0] * 1000
        robotPose[1] = robotPose[1] * 1000
        robotPose[2] = robotPose[2] * 1000
        self.tprint('Robot pose is (rad): %s' %str(robotPose))
        robotPose[3] = np.rad2deg(robotPose[3])
        robotPose[4] = np.rad2deg(robotPose[4])
        robotPose[5] = np.rad2deg(robotPose[5])
        self.tprint('Robot pose is : %s' %str(robotPose))

        self.robot_pose = robotPose
        return robotPose
    
    def robotSetGripperPose(self):
        # set pose
        self.tprint('Robot set pose GUI ... ') 
        
        robotPose = self.robotGetGripperPose()
        poseGui   = PoseGUI(robotPose)
        robotPose = poseGui.pose
   
        self.tprint('Robot pose from user input : %s' %str(robotPose))

        self.robotAbsoluteMovePose(robotPose)

        return robotPose
       
        
    def robotMarkWorkPosition(self):
        # read pose
        self.tprint('Robot shows work area ... ') 

        
        val = 0
        for robotPose in WORK_POINTS:
            print(f"Перемещение к: {robotPose}")
            self.rbm.setmovel(robotPose)
            time.sleep(0.5)
            self.rbm.setDO(1,val)
            print('Robot gripper command %d ... ' %val) 
            val = 1 - val

    def robotGoHome(self):
        # read pose
        self.tprint('Robot home pose ... ') 
               
        # res = self.rbm.getmovel() #getTool_xyzrxryrz()
        # isOk, robotPose, msg = res
        # if not isOk:
        #     return

        robotPose = self.home_pose #[-0.689, -0.121, 1.035, -2.0, 0.0, 2.0]
        self.robotAbsoluteMovePose(robotPose)

    def robotSetHomePose(self):
        # setting home pose
        self.tprint('Robot set current pose to be home pose ... ') 
        
        robotPose = self.robotGetGripperPose()        
        self.home_pose[0] = robotPose[0]
        self.home_pose[1] = robotPose[1]
        self.home_pose[2] = robotPose[2]

    def robotGripperOnOff(self,val = 1):
        # set gripper
        if val < 0.5:
            self.rbm.SetGripper('close')
        else:
            self.rbm.SetGripper('open')
            
        self.tprint('Robot gripper command %d.' %(val)) 
        
    def robotDiffMovePose(self, dPose = np.zeros((1,6))):
        # read pose
        # maybe already running
        if self.rbm is None or self.rbm.connected() is False:
            self.tprint('Connect to the robot first ...')
            return

        self.tprint('Robot read pose and move ... ')         
        
        res = self.rbm.getmovel() #getTool_xyzrxryrz()
        isOk, robotPose, msg = res
        self.tprint('Robot pose is : %s' %str(np.round(robotPose,2)))
        if not isOk:
            self.tprint('Can not read position')
            return
        
        # dPose[0] = dPose[0] / 1000
        # dPose[1] = dPose[1] / 1000
        # dPose[2] = dPose[2] / 1000
        # dPose[3] = np.deg2rad(dPose[3])
        # dPose[4] = np.deg2rad(dPose[4])
        # dPose[5] = np.deg2rad(dPose[5])

        robotPose[0] = robotPose[0] + dPose[0]
        robotPose[1] = robotPose[1] + dPose[1]
        robotPose[2] = robotPose[2] + dPose[2]
        robotPose[3] = robotPose[3] + dPose[3]
        robotPose[4] = robotPose[4] + dPose[4]
        robotPose[5] = robotPose[5] + dPose[5]
        
        self.tprint('Going to Robot pose %s ' %str(robotPose))
        self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2= str(self.robot_speed), num3= str(self.robot_speed))
        
    def robotAbsoluteMovePose(self, dPose = [0]*6):
        # move to pose

        # maybe already running
        # if self.rbm is None or self.rbm.connected() is False:
        #     self.tprint('Connect to the robot first ...')
        #     return 

        #self.tprint('Robot read pose and move ... ') 
        self.tprint('Going to Robot pose [mm,deg] %s ' %str(np.round(dPose,2)))

        # res = self.rbm.getmovel() #getTool_xyzrxryrz()
        # isOk, robotPose, msg = res
        # #self.tprint('Robot pose is : %s' %str(robotPose))
        # if not isOk:
        #     self.tprint('Can not read position')
        #     return
        robotPose    = dPose.copy() #np.array(dPose).copy()
        
        robotPose[0] = dPose[0] / 1000
        robotPose[1] = dPose[1] / 1000
        robotPose[2] = dPose[2] / 1000
        robotPose[3] = np.deg2rad(dPose[3])
        robotPose[4] = np.deg2rad(dPose[4])
        robotPose[5] = np.deg2rad(dPose[5])

        
        #self.tprint('Going to Robot pose [m,rad]%s ' %str(robotPose))
        #self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2='100', num3='100')
        #isOk, robotPose, msg = self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2= str(self.robot_speed), num3= str(self.robot_speed))
        self.rbm.setmovel(robotPose)
        return 
    
    
    
    ## ------------------------------------  
    # -- Host Control ---
    ## ------------------------------------ 

    def hostStartPose6D(self):
        # start
        self.tprint('Start host ...')
        
        cwd = os.getcwd()
        os.chdir(r'..\..\Pose6D')
        os.startfile("..\..\Pose6D\Pose6D-1915.exe")
        os.chdir(cwd)

    def hostConnect(self):
        # start
        self.tprint('Starting host connection ...')
        
        # maybe already running
        if self.vis is None:
            #print(self.vis)
            # runs Robot server and Multi Object Detection
            self.vis    = HostManager(host = self.ip_vision, port = self.port_vision, debug = self.debugOn, config=self.cfg) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
            self.vis.start()
        elif self.vis.isAlive():   
            self.tprint('Pose6D Connection is alive')
        else:
            #self.vis.start()
            self.tprint('Vision Connection is restarted')
            
    def hostStop(self):
        # stop
        self.tprint('Stop host  ...')
        self.vis.stop()
            
    def hostStatus(self):
        # start
        self.tprint('Getting host status ...')

    def hostSendRecv(self, objId=1,objNum=1,robotPose=np.zeros((1,6)),objQ=1, msgId=1):
        # send receive info
        # use msgId for stereo messages
        
        self.tprint('Client sending request to host ...')  
        #self.vis.setRobotData(objId,objNum,robotPose,objQ)
        self.vis.setObjectData(objId,robotPose,objQ,msgId)
        
        time.sleep(0.1)    
        #objId,objNum,objPose,objQ = self.vis.getObjectData()  
        objId,objPose,objQ = self.vis.getObjectData() 
        if objId is None:
            self.tprint('Client Rx problem')
        else:            
            self.tprint('Client Receives  : %s,%s,%s,%s' %(str(objId),str(objNum),str(np.round(objPose.ravel(),2)),str(objQ)))
        
        return objId,objNum,objPose.flatten(),objQ
    
    def hostDetectObject(self, robotPose = []):
        # detect tool
        self.tprint('Detecting object pose ...')
        if not self.vis.isAlive():
            self.tprint('Vision task is not alive','W')
            return
        
        if len(robotPose) < 1:
            robotPose = self.robot_pose

        self.tprint('Robot pose is : %s' %str(robotPose))
        
        # simple protocol
        robId,robNum,robPose,robQ = 1,1, np.array(robotPose),1
        objId,objNum,objPose,objQ = self.hostSendRecv(robId,robNum,robPose,robQ)
        
        # use object list internally
        #self.dsp.UpdateObject(None, obj_name = '1', obj_num = 1, obj_pose = objPose )

        if objQ < 0.7:
            self.tprint('Object quality is low : %s' %str(objQ))
            objPose = robPose

        objPose          = np.round(objPose,decimals=3).astype(float)
        self.object_pose = objPose
        self.tprint('Object pose is : %s' %str(objPose))
        return objPose
    

    def hostDetectTool(self):
        # detect tool
        self.tprint('Detect tool ...')
        if not self.vis.isAlive():
            return
        
        robId,robNum,robPose,robQ = 'tool',1, np.array([0,0,0,0,0,0]),1
        objId,objNum,objPose,objQ = self.hostSendRecv(robId,robNum,robPose,robQ)
        
        # use object list internally
        self.dsp.UpdateObject(None, obj_name = 'tool', obj_num = 1, obj_pose = objPose )
        

    def hostDetectScrew(self):
        # start
        self.tprint('Detect screws ...')
        if not self.vis.isAlive():
            return
        
        robId,robNum,robPose,robQ = 'screw',self.sliderCount, np.array([0,0,0,0,0,0]),1
        objId,objNum,objPose,objQ = self.hostSendRecv(robId,robNum,robPose,robQ)        

        self.tprint('Vision is stopped')   
        
    def hostSimServerTask(self, k=0):
        # Update info about the objects 
        self.tprint('Simulator thread is running') 
        self.time_start = time.time()
        time_period     = 20
        while not self.sim.stopped:
            # Read temperature
            time_now    = time.time() - self.time_start
            x_pos       = np.cos(2*np.pi*time_now/time_period)*200 + 500
            y_pos       = np.sin(2*np.pi*time_now/time_period)*200 + 500

            # server receives
            robId,robNum,robPose,robQ =  self.sim.getRobotData()
            if robId is not None:
                self.tprint('Server Receives  : %s,%s,%s,%s' %(str(robId),str(robNum),str(np.round(robPose.ravel(),2)),str(robQ)))

            time.sleep(0.01)

            # server response
            if robId is not None:
                robPose[0], robPose[1] = x_pos, y_pos
                objId, objNum, objPose, objQ   = robId, robNum, robPose, 0.95 
                self.tprint('Server Sending   : %s,%s,%s,%s' %(str(objId),str(objNum),str(np.round(objPose.ravel(),2)),str(objQ)))                 
                self.sim.setObjectData(objId, objNum, objPose, objQ)
            
            time.sleep(0.01)

            
        self.tprint('Simulator thread is stopped')         
        
    def hostSimServer(self):
        # start
        self.tprint('Simulating host connection ...')
        
        # maybe already running
        if self.sim is None:
            # runs Robot server and Multi Object Detection
            self.sim    = hostSimulator(host = self.ip_vision, port = self.port_vision,  debug=self.debugOn)
        #elif  not self.com.isAlive():      
        self.sim.start() 
        
        # run the task to create data
        self.btnAcquireStatus = True
        
        self.sim_ts = Thread(target=self.hostSimServerTask)
        self.sim_ts.start()  
        
        
    def hostDetectLeaf(self):
        # start
        self.tprint('Detect Leafs ...')

    def hostDetectGrape(self):
        # start
        self.tprint('Detect Leafs ...')

    def hostDisConnect(self):
        # start
        self.tprint('Disconnect from host ...')
        
        if self.sim is not None:  
            self.sim.stop()       
            
        # maybe already running
        if self.vis is not None:
            self.vis.stop()
            self.tprint('Stopping host thread...')        
        
    ## ------------------------------------  
    # -- IO Control ---
    ## ------------------------------------ 
    def ioConnect(self):
        # start
        self.tprint('Starting IO connection ...')
        
        # maybe already running
        if self.ioc is None:
            # runs Robot server and Multi Object Detection
            self.ioc    = IOController(self) #host = self.ip_vision, port = self.port_vision, debug = self.debugOn, config=self.cfg) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
            self.ioc.ConnectToController()
        elif self.ioc.IsConnected():   
            self.tprint('IO Connection is alive')
        else:
            # need to connect
            self.ioc.ConnectToController()
            self.tprint('IO Connection is initiated')
            
    def ioStatus(self):
        self.ioc.GetInputStatus()
        self.tprint(f'Checking status')
        
    def ioReset(self):
        # get specific bit info
        addr = 0
        self.ioc.ResetOutput(addr)
        self.tprint(f'IO reset {addr}') 
        
    def ioGetInfo(self):
        # get specific bit info
        val = self.ioc.GetInputStatus(0)
        self.tprint(f'IO received {val}')
        
    def ioSetInfo(self):
        # get specific bit info  
        addr = 0
        val = 1
        self.ioc.SetOutput(addr,val)
        self.tprint(f'IO sending value {val} to {addr}')
        
    def ioDisconnect(self):
        # disonnecteing
        self.ioc.CloseConnectionWithController()        
       
    ## ------------------------------------  
    # -- Task --      
    ## ------------------------------------ 
        
    def robotMultiPointMotion(self):
        # check multi point motion as deefined by China
        self.tprint('Starting point motion ...')

        val = 1
        pointNum  = len(SCAN_POINTS)
        for k in range(pointNum):

            #self.robotCommState()
            robotPose = SCAN_POINTS[k].copy()

            self.tprint('Moving....')
            self.robotAbsoluteMovePose(robotPose)
            time.sleep(0.1)

            self.tprint('Gripper on-off')
            self.robotGripperOnOff(val)
            val = 1 - val

            self.tprint('Finishing scan point %d from %d' %(k+1,pointNum))
            

        self.tprint('Robot finished the scan.')


    def robotDetectAndMoveToPoint(self):
        # start
        #self.tprint('Client sending request to robot. Go Home Pose ...')
        #self.robotAbsoluteMovePose([-356,-654,285,-90,0,140])

        self.tprint('Client sending request to robot ...') 
        robotPose = self.robotGetGripperPose()

        #self.tprint('Client sending request to host ...') 
        objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPose)
        self.tprint('Received Object pose [mm,deg] %s ' %str(np.round(objPose,2)))
        if objQ < 0.7:
            self.tprint('Object is not detected. Not moving.')
            return

        self.tprint('Client sending Command to the Robot Move Linear ...')
        #objPose[0] = objPose[0] + SAFETY_MARGIN_Z # actually X
        self.robotAbsoluteMovePose([objPose[0],objPose[1],objPose[2],robotPose[3],robotPose[4],robotPose[5]])
        # self.robotAbsoluteMovePose([-400,-700,robotPose[2],robotPose[3],robotPose[4],robotPose[5]])

        self.tprint('Robot is at object pose - check.')

    def robotScanDetectMove(self):
        # scan multiple position detect and move to the points
        self.tprint('Starting scan %s times...' %str(self.sliderCount))

        pointNum  = len(SCAN_POINTS)
        for repeatNum in range(self.sliderCount):
            self.tprint('Repeat %s ...' %str(repeatNum))
            for k in range(pointNum):

                self.tprint('Going to point %d ....' %(k+1))
                self.robotAbsoluteMovePose(SCAN_POINTS[k])
                #time.sleep(15)

                self.tprint('Touch....')
                self.robotDetectAndMoveToPoint()
                #time.sleep(10)

                self.tprint('Going from point %d ....' %(k+1))
                self.robotAbsoluteMovePose(SCAN_POINTS[k])
                #time.sleep(10)
                self.tprint('Finishing scan point %d from %d' %(k+1,pointNum))
            

        self.tprint('Robot finished the scan.')

    def robotDetectMoveAndRepeat(self):
        # touches the point several times
        SMALL_DIFF_POSE    = [[ 0,  0, 0, 0 ,0, 0],
                              [ 0, -50, 0, 0, 0, 0],
                              [ 50,-50, 0, 0, 0, 0],                              
                              [ 50,  0, 0, 0, 0, 0]]
        pointNum = len(SMALL_DIFF_POSE)
        self.tprint('Repeating %d times ...' %pointNum) 
        robotPose = self.robotGetGripperPose()

        for k in range(pointNum):
            self.tprint('Client sending request to robot ...') 
            robotPoseTemp = robotPose.copy()
            robotPoseDiff = SMALL_DIFF_POSE[k]
            robotPoseTemp[0] += robotPoseDiff[0]
            robotPoseTemp[1] += robotPoseDiff[1]
            self.robotAbsoluteMovePose(robotPoseTemp)
            time.sleep(0.5)

            #self.tprint('Client sending request to host ...') 
            objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPoseTemp)
            self.tprint('Received Object pose [mm,deg] %s ' %str(np.round(objPose,2)))
            if objQ < 0.7:
                self.tprint('Object is not detected. Not moving.')
                break

            self.tprint('Client sending Command to the Robot Move Linear ...')
            #objPose[0] = objPose[0] + SAFETY_MARGIN_Z # actually X
            self.robotAbsoluteMovePose([objPose[0],objPose[1],objPose[2],robotPose[3],robotPose[4],robotPose[5]])
            # self.robotAbsoluteMovePose([-400,-700,robotPose[2],robotPose[3],robotPose[4],robotPose[5]])

            self.tprint('Robot is at object pose - check.') 

        self.robotAbsoluteMovePose(robotPose)       

    def taskStereoMoveToPoint(self):
        # takes several positions and compute stereo
        STEREO_DIFF_POSE   = [[ 0,   0, 0, 0 ,0,0],
                              [ 0,-50, 0, 0, 0,-5],
                              [ 0, 50, 0, 0, 0, 5]]

        self.tprint('Get current robot pose ...') 
        robotPose = self.robotGetGripperPose()
        
        self.tprint('Init stereo...')
        objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPose, msgId= 11)
        
        pointNum  = len(STEREO_DIFF_POSE)
        for k in range(pointNum):
            
            robotPoseTemp = robotPose.copy()
            robotPoseDiff = STEREO_DIFF_POSE[k]
            robotPoseTemp[1] += robotPoseDiff[1]
            robotPoseTemp[5] += robotPoseDiff[5]
            #robotPoseTemp  = [robotPose[k] += STEREO_DIFF_POSE[k] for k in range(6)]

            self.tprint('Going to point %d ....' %(k+1))
            self.robotAbsoluteMovePose(robotPoseTemp) 
            time.sleep(0.5)

            self.tprint('Client sending request to stereo host view %d...' %(k+1)) 
            objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPoseTemp, msgId= 13)
            if objQ < 0.7:
                self.tprint('Object is not detected. Not moving.')
                continue
            
        # return robot to the initial pose
        self.robotAbsoluteMovePose(robotPose) 
            
        self.tprint('Client sending request to stereo host. Triangulating...') 
        objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPose, msgId= 17)
        self.tprint('Received Object pose [mm,deg] %s ' %str(np.round(objPose,2)))
        if objQ < 0.7:
            self.tprint('Object is not detected. Not moving.')
            return            

        time.sleep(0.5)
        self.tprint('Client sending Command to the Robot Move Linear ...')
        #objPose[0] = objPose[0] + 300 #SAFETY_MARGIN_Z # actually X
        self.robotAbsoluteMovePose([objPose[0],objPose[1],objPose[2],robotPose[3],robotPose[4],robotPose[5]])
        # self.robotAbsoluteMovePose([-400,-700,robotPose[2],robotPose[3],robotPose[4],robotPose[5]])
        self.tprint('Robot is at object pose - check.')
           

    # show on the gui
    def showAllHangers(self):
        # read all the nagers and show them
        self.dsp.ClearScene()

#        self.getHangerNumber()
#        hangerNum           = self.msgRecv['Data']
#        self.tprint('Reading %d hanger poses...' %int(hangerNum))
#        v1                  = np.array([-100.0,  1000.0,   30.0,   0.0, 0.0, -0.0 ]).reshape(1,6)
#        extrinsics_obj      = np.vstack((v1))    
#        hanger_list          = self.dsp.GetHangerParams(extrinsics_obj)
#
#        for k in range(hangerNum):
#            
#            self.sliderCount = k+1
#            self.getHangerPose()
#           
#            # update the pose - only X values
#            hangerPose                         = self.msgRecv['Data']
#            hanger_list[0].extrinsics[1]       = hangerPose[1] 
#                
#            hanger_list = self.dsp.DrawObjects(self.ax, hanger_list)
#                    
#            self.fig.canvas.draw()
#            self.fig.canvas.flush_events()
#
#        self.tprint('Reading is done...')    
    
    ## -----------------------------
    # -- Display Control ---
    ## -----------------------------
    
    def renderScene(self):
        # creates new scene 
        self.dsp.TestInitShowCameraTableTool(self.ax, self.fig)
        self.tprint('Robot new scene rendering : %s' %str('A')) 
        
    def renderSceneMotion(self):
        # creates new scene 
        self.dsp.TestInitShowCameraTableTool(self.ax, self.fig)
        self.tprint('Motion is started : %s' %str('A')) 
        
    def renderSceneWithGrapes(self):
        # creates new scene 
        self.dsp.TestInitShowCameraGripperPoints(self.ax, self.fig)
        self.tprint('Grape scene : %s' %str('A'))         
        
                             
        

    ## -----------------------------            
    # -- Help  ---
    ## -----------------------------
        
    def helpSystemStatus(self):
        # start
        self.tprint('System status  ...')        
        
    def helpUserManual(self):
        # start
        self.tprint('For User Manual - conatct RobotAI  ...')



    # -- Service ---        
    
    def debugOnOff(self):
        # switch debug
        self.debugOn        = not self.debugOn
        self.com.debugOn    = self.debugOn
        self.tprint('Debug switch : %d' %self.debugOn)          



    def finish(self):
        # stop tasks
        self.btnAcquireStatus = False
        
        #self.hostDisConnect()
                       
        self.tprint('Done')
        self.mainQuit()
        #self.win.destroy()
        #self.win.quit()  
        #sys.exit(1)
        
    def mainQuit(self):
        # correct way to close
        #breakpoint()
        self.win.quit()     # stops mainloop
        self.win.destroy()  # this is necessary on Windows to prevent
                            # Fatal Python Error: PyEval_RestoreThread: NULL tstate        
    
    # -------------------------
    def setupMenu(self):
        # add menu
        menu = tk.Menu(self.win)
        self.win.config(menu=menu)

        # ---------------------------------------
        # Session
        boardmenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='Setup',  menu=boardmenu)
#        boardmenu.add_command(label='Connect...',               command= self.commServer)
        boardmenu.add_command(label='Status...',                command= self.hostStatus)
        boardmenu.add_command(label='Info...',                  command= self.getVersion)
        #boardmenu.add_separator()
        #boardmenu.add_command(label='Disconnect...',            command= self.commDisConnect)
        boardmenu.add_separator()
        boardmenu.add_command(label='Edit Session...',          command= self.sessionSelect)

        # ---------------------------------------
        # Main Control
        ordermenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='Control',  menu=ordermenu)
        ordermenu.add_command(label='Connect...',                command=self.orderSelect)
        ordermenu.add_separator()
        ordermenu.add_command(label='Select order',              command=self.orderSelect)

        # ---------------------------------------
        # Robot commands
        configmenu = tk.Menu(menu,tearoff = 0 )  
        testmenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='Robot',  menu=testmenu)
        testmenu.add_command(label='Connect...',                  command= self.robotConnect)
        testmenu.add_command(label='Status...',                   command= self.robotStatus)
        testmenu.add_command(label='Stop...',                     command= self.robotStop)
        testmenu.add_command(label='Disconnect...',               command= self.robotDisConnect)
        testmenu.add_separator()
      
        testmenu.add_command(label='Get Gripper Pose',            command=self.robotGetGripperPose)
        testmenu.add_command(label='Set Gripper Pose',            command=self.robotSetGripperPose)
        
        testmenu.add_separator()
        testmenu.add_command(label='Go Home',                     command=self.robotGoHome)
        testmenu.add_command(label='Set Home Pose to Current',    command=self.robotSetHomePose)      
        testmenu.add_command(label='Set Robot Speed',             command=self.robotSetRobotSpeed)
        testmenu.add_command(label='Gripper On/Off',              command=self.robotGripperOnOff)      
        testmenu.add_command(label='Show Work Position',          command=self.robotMarkWorkPosition)
        testmenu.add_command(label='Command And Go',              command=self.robotDetectAndMoveToPoint)
        testmenu.add_command(label='Execute Pick',                 command=self.robotDetectAndMoveToPoint)

        testmenu.add_separator()
        testmenu.add_cascade(label='Diff move...',              menu = configmenu)             
        configmenu.add_command(label='+dX',                     command=lambda:self.robotDiffMovePose([ TX_STEP_SIZE,0,0,0,0,0])) 
        configmenu.add_command(label='-dX',                     command=lambda:self.robotDiffMovePose([-TX_STEP_SIZE,0,0,0,0,0]))
        configmenu.add_command(label='+dY',                     command=lambda:self.robotDiffMovePose([0, TX_STEP_SIZE,0,0,0,0])) 
        configmenu.add_command(label='-dY',                     command=lambda:self.robotDiffMovePose([0,-TX_STEP_SIZE,0,0,0,0]))
        configmenu.add_command(label='+dZ',                     command=lambda:self.robotDiffMovePose([0,0, TX_STEP_SIZE,0,0,0])) 
        configmenu.add_command(label='-dZ',                     command=lambda:self.robotDiffMovePose([0,0,-TX_STEP_SIZE,0,0,0]))
        configmenu.add_command(label='+rX',                     command=lambda:self.robotDiffMovePose([0,0,0,1.57,0,0]))
        configmenu.add_command(label='-rX',                     command=lambda:self.robotDiffMovePose([0,0,0,-1.57,0,0]))
        configmenu.add_command(label='+rY',                     command=lambda:self.robotDiffMovePose([0,0,0,0,0.57,0]))
        configmenu.add_command(label='-rY',                     command=lambda:self.robotDiffMovePose([0,0,0,0,-0.57,0]))     
        configmenu.add_command(label='+rZ',                     command=lambda:self.robotDiffMovePose([0,0,0,0,0,0.57]))
        configmenu.add_command(label='-rZ',                     command=lambda:self.robotDiffMovePose([0,0,0,0,0,-0.57]))                 
        #testmenu.add_separator()
        #testmenu.add_command(label='Simulator...',               command= lambda: self.commClient())

        # ---------------------------------------
        # Host Control
        pose6dmenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='Host',  menu=pose6dmenu)
      
        pose6dmenu.add_command(label='Connect...',                command=self.hostConnect)
        pose6dmenu.add_command(label='Status...',                 command= self.robotStatus, state="disabled")
        pose6dmenu.add_command(label='Stop...',                    command= self.robotStop, state="disabled")
        pose6dmenu.add_command(label='Disconnect...',              command=self.hostDisConnect)
        
        pose6dmenu.add_separator()
        pose6dmenu.add_command(label='Send Info',                 command=self.hostDetectObject)  
        pose6dmenu.add_command(label='Send Status',               command=self.hostDetectLeaf, state="disabled")        
    
        # ---------------------------------------- 
        # IO
        iomenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='IO',  menu=iomenu)
      
        iomenu.add_command(label='Connect...',                   command=self.ioConnect)
        iomenu.add_command(label='Status...',                    command= self.ioStatus)
        iomenu.add_command(label='Rest...',                      command= self.ioReset)
        iomenu.add_command(label='Disconnect...',                command=self.ioDisconnect)
        
        iomenu.add_separator()
        iomenu.add_command(label='Get Info',                     command=self.ioGetInfo)  
        iomenu.add_command(label='Set Value',                    command=self.ioSetInfo)          

        # ----------------------------------------
        # Task commands
        taskmenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='Tasks',  menu=taskmenu)
        taskmenu.add_command(label='Test Multi Point Motion',      command=self.robotMultiPointMotion)
        taskmenu.add_command(label='Detect Object and Move',       command=self.robotDetectAndMoveToPoint)
        taskmenu.add_command(label='Scan, Detect and Move',        command=self.robotScanDetectMove) 
        taskmenu.add_separator() 
        taskmenu.add_command(label='Detect, Move and Repeat',       command=self.robotDetectMoveAndRepeat)  
        taskmenu.add_separator()  
        taskmenu.add_command(label='Stereo Detect and Move',        command=self.taskStereoMoveToPoint)  
        taskmenu.add_separator()     
        taskmenu.add_command(label='Test Precision',               command=self.helpUserManual, state="disabled")
        
        
        # ----------------------------------------
        # Graphics commands
        # setup varaibles
        self.showHangers = tk.BooleanVar()
        self.showHangers.set(True)
        self.showSeparatos = tk.BooleanVar()
        self.showLeafs = tk.BooleanVar()

        graphmenu = tk.Menu(menu,tearoff = 0 )
        menu.add_cascade(label='View',  menu=graphmenu)
        graphmenu.add_checkbutton(label="Show Pose",                onvalue=1, offvalue=0,    variable=self.showHangers)
        graphmenu.add_checkbutton(label="Show IO",                  onvalue=1, offvalue=0,    variable=self.showSeparatos)
        graphmenu.add_checkbutton(label="Show Status",              onvalue=1, offvalue=0,    variable=self.showLeafs)
        graphmenu.add_separator()
        graphmenu.add_command(label='Show robot pose',              command=self.showAllHangers)
        graphmenu.add_separator()
        graphmenu.add_command(label='Render scene A',               command=self.renderScene)
        graphmenu.add_command(label='Simulate Motion',              command=self.renderSceneMotion)
        graphmenu.add_command(label='Render Scene B',               command=self.renderSceneWithGrapes)
        #viewmenu = tk.Menu(menu,tearoff = 0 )
        #menu.add_cascade(label='View', menu=viewmenu)
        #self.win.config(menu=viewmenu)

        
        # Help
        helpmenu = tk.Menu(menu, tearoff = 0)
        menu.add_cascade(label='Help',        menu=helpmenu)
        helpmenu.add_command(label='User Manual',               command=self.helpUserManual)
        helpmenu.add_command(label='About/Version',             command=self.getVersion)
        helpmenu.add_command(label='Debug On/Off',              command=self.debugOnOff)        
        helpmenu.add_separator()
        helpmenu.add_command(label='Exit',                      command=self.finish)


        
    def setupPlot(self):
        # https://matplotlib.org/stable/gallery/user_interfaces/embedding_in_tk_sgskip.html

        self.fig  = Figure(figsize=(10, 6), dpi=100)
        self.t    = np.arange(0, 2000, 1)
        self.y    = 1.5 + 0*self.t
        self.ax   = self.fig.add_subplot(111)
        self.line, = self.ax.plot(self.t, self.y)
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("s A")
                
        # run multiple
        self.x_data = []
        self.y_data = []
        
    def setupPlot3D(self):
        # https://matplotlib.org/stable/gallery/user_interfaces/embedding_in_tk_sgskip.html

        self.fig        = Figure(figsize=(10, 6), dpi=100)
        #self.fig.canvas.set_window_title('3D Scene')
        self.z_data     = np.arange(0, 1000, 1)
        self.y_data     = np.sin(self.z_data)*500 + 500
        self.x_data     = np.cos(self.z_data)*500 + 500
        
#        self.ax         = self.fig.subplots() #
#        self.line       = self.ax.plot(self.x_data, self.y_data, self.z_data)
#        
#        self.ax         = self.fig.add_subplot()
#        self.line       = self.ax.plot(self.x_data, self.y_data, self.z_data)
        
        # matplotlib 3.5.3
        self.ax         = self.fig.gca(projection='3d') #self.fig.add_subplot(111) # #self.fig.add_subplot(111)
        #self.ax         = self.fig.add_subplot(projection='3d')
        self.line,      = self.ax.plot(self.x_data, self.y_data, self.z_data)
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.ax.set_zlabel("Z [mm]")
        #Axes3D.mouse_init(self.ax)
        #self.fig.tight_layout()
        self.ax.xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        self.ax.yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        self.ax.zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        #self.ax.axis('equal')
        #self.ax.axis("off")
        #self.ax.view_init(elev=90, azim=0)
                
        # run multiple
        self.x_data = []
        self.y_data = []    
        self.z_data = []
    
        

    def setupControls(self):
        # https://matplotlib.org/stable/gallery/user_interfaces/embedding_in_tk_sgskip.html
        
        # Create left and right frames
        left_frame  =  tk.Frame(self.masterframe,  width=60,  height=400,  bg='black')
        left_frame.pack(side='left',  fill='both',  padx=10,  pady=5,  expand=True)

        right_frame  =  tk.Frame(self.masterframe,  width=820,  height=400,  bg='black')
        right_frame.pack(side='right',  fill='both',  padx=10,  pady=5,  expand=True)        
        
        self.canvas  = FigureCanvasTkAgg(self.fig, master=right_frame)
        #self.canvas.get_tk_widget().grid(row=1,column=0) #,rowspan = 4, sticky="NSEW") #,columnspan = 4,rowspan = 4)
        self.canvas.draw()
        #self.canvas.set_window_title('3D Scene')
        self.right_frame = right_frame
        
        # pack_toolbar=False will make it easier to use a layout manager later on.
        #toolbar = NavigationToolbar2Tk(self.canvas, right_frame) #, pack_toolbar=False)
        #toolbar.update()
#        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        #self.canvas.mpl_connect("key_press_event", lambda event: print(f"you pressed {event.key}"))
        #self.canvas.mpl_connect("key_press_event", key_press_handler)

        #button_quit = tk.Button(master=left_frame, text="Quit", command=self.win.destroy, relief=tk.RAISED)
        
        # mode_menu =  tk.Menubutton ( left_frame, text="Modes",  relief=tk.RAISED )
        # mode_menu.menu =  tk.Menu ( mode_menu, tearoff = 0 )
        # mode_menu["menu"] =  mode_menu.menu

        # mayoVar = tk.IntVar()
        # ketchVar = tk.IntVar()

        # mode_menu.menu.add_checkbutton ( label="mayo",  variable=mayoVar )
        # mode_menu.menu.add_checkbutton ( label="ketchup",  variable=ketchVar )
        
        variable            = tk.StringVar(left_frame)
        variable.set(AXIS_VIEW_OPTIONS[0]) # default value
        mode_menu           = tk.OptionMenu(left_frame, variable, *AXIS_VIEW_OPTIONS, command = self.menuViewOptons)
        #def ok():
        #    print ("value is:" + variable.get())

        #mode_button = tk.Button(left_frame, text="OK", command=ok) self.robotStop
        #combo_box = ttk.Combobox(values=["Python", "C", "C++", "Java"]) 
        
        self.btnStatus      = tk.Button(master=left_frame, text="Robot Go Home",    command=self.robotGoHome, relief=tk.RAISED)
        self.btnAcquire     = tk.Button(master=left_frame, text="Robot Pose",       command=self.robotGetGripperPose, relief=tk.RAISED) # btnAcquirePress
        self.btnAccel       = tk.Button(master=left_frame, text="Object Pose   ",   command=self.hostDetectObject, relief=tk.RAISED)   # btnAccelPress
        self.btnEcgRaw      = tk.Button(master=left_frame, text="Robot Grab",       command=self.robotDetectAndMoveToPoint, relief=tk.RAISED) # btnEcgRawPress
        self.btnFileOffLine = tk.Button(master=left_frame, text="File Offline",     command=self.btnFileOffLinePress, relief=tk.RAISED)
        self.btnFileOnLine  = tk.Button(master=left_frame, text="File on Board",    command=self.btnFileOnLinePress, relief=tk.RAISED)
        self.status         = tk.Entry(master=right_frame)
        button_quit         = tk.Button(master=left_frame, text="Quit",                 command=self.finish, relief=tk.RAISED)
        
        slider_update       = tk.Scale(left_frame, label="Count", from_=1, to=200, orient=tk.HORIZONTAL, command=self.sliderUpdate)
        
                
        self.ledAbutton     = tk.Button(master=left_frame, text="Grip On/Off", fg="white", bg="black", command = self.btnA)
        self.ledBbutton     = tk.Button(master=left_frame, text="STOP", fg="white", bg="red", command = self.robotStop) # btnB

        
        # Packing order is important. Widgets are processed sequentially and if there
        # is no space left, because the window is too small, they are not displayed.
        # The canvas is rather flexible in its size, so we pack it last which makes
        # sure the UI controls are displayed as long as possible.
        
        ipadding = {'ipadx': 10, 'ipady': 10, 'padx': 5, 'pady': 5}
        #toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        mode_menu.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        #mode_button(**ipadding, side=tk.TOP, fill=tk.BOTH)        
        
        self.btnStatus.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.btnAcquire.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.btnAccel.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.btnEcgRaw.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.btnFileOffLine.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.btnFileOnLine.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        slider_update.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        #combo_box.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
  
        self.ledAbutton.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        self.ledBbutton.pack(**ipadding, side=tk.TOP, fill=tk.BOTH)
        button_quit.pack(**ipadding, side=tk.BOTTOM, fill=tk.BOTH)
                
        self.status.pack(**ipadding, side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        self.status.delete(0,tk.END)
        self.status.insert(0,'Parameters and Messages')
        
        # toolbar
        #toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)  

        
        # run multiple
        self.btnAcquireStatus = False
        
    def setupView(self):
        # initial view

        self.win.minsize(width=920, height=420)
        self.win.config(bg = 'gray')
        self.win.title("PRIMATIC")
        
        fpath = iconFilePath if os.path.isfile(iconFilePath) else "logo.ico"
        self.win.iconbitmap(fpath) # put it here because of comm

        self.masterframe = tk.Frame(bg = "gray")
        self.masterframe.pack(side='top',fill='both',expand=True)
        self.masterframe.grid_columnconfigure(0, weight=1)
        self.masterframe.grid_rowconfigure(0, weight=1)

        # add menu & controls
        self.setupMenu()
        # setup plot axis before controls
        #self.setupPlot()
        self.setupPlot3D()
        self.setupControls()
        
        # make DisplayManager draw the scene
        self.line.remove()
        self.dsp.TestInitShowCameraGripperPoints(self.ax, self.fig)
        #self.dsp.TestRenderaGripperHangersSeparators(self.ax)
        #self.commServer()
        self.startAuto()

    def startAuto(self):
        # start auto connections
        #self.robotConnect()
        #self.robotStatus()
        #self.hostConnect()
        #self.hostStatus()
        pass

        
    # ---------------------------------
    # -- Change 3D view
    def menuViewOptons(self, inp):
        self.tprint('View menu :  %s' %str(inp))
        
        if inp == AXIS_VIEW_OPTIONS[0]: # 45-45
            self.ax.view_init(elev=45, azim=45) 
        elif inp == AXIS_VIEW_OPTIONS[1]: # LEFT
            self.ax.view_init(elev=5, azim=90) 
        elif inp == AXIS_VIEW_OPTIONS[2]: # TOP
            self.ax.view_init(elev=90, azim=0)       
            
        self.canvas.draw() # refresh the view
        
        
        
    # ----------------------------------------    
    # -- Task --
    def acquirePositionRealTime(self, k=0):
        # Update subplots 
        self.time_start = time.time()
        # clean the screen
        self.dsp.ClearScene()
        v1                  = np.array([100.0,  200.0,   300.0,   0.0, 0.0, -0.0 ]).reshape(1,6)
        extrinsics_obj      = np.vstack((v1))    
        griper_list         = self.dsp.GetGripperParams(extrinsics_obj)        
        
        while self.btnAcquireStatus:
            #self.tprint('%s' %k)
            # get hanger position
            self.getGripperPose()
           
            # update the pose - only X values
            gripPose = self.msgRecv['Data']
            #print(type(gripPose))
            #self.dsp.TestRenderaGripperHangersSeparators(self.ax,[hangerPose[0]],[])
            
            griper_list[0].extrinsics[:3]       = gripPose[:3] 
                
            griper_list = self.dsp.DrawObjects(self.ax, griper_list) 
            
            # required to update canvas and attached toolbar!
            self.canvas.draw()   
            time_now    = time.time() - self.time_start                      
            time.sleep(0.01)  
            
        self.tprint('Exiting...')
            
    # -- Control --            
    def btnAcquirePress(self):
        # when pressed starts or stops thread
        if self.btnAcquireStatus == False:
            self.btnAcquireStatus = True
            self.btnAcquire.config(bg="red", fg="white", text="Stop")
            
            self.ts = Thread(target=self.acquirePositionRealTime)
            #self.ts.daemon = True
            self.ts.start()  
            self.tprint('Thread is running')   
            
        else:
            self.btnAcquireStatus = False
            self.btnAcquire.config(bg="gray", fg="black", text="Gripper Pose")
            #self.ts.join()
            self.tprint('Thread is stopped')             
       
    # ---------------------------------         
    # -- Configure --
    def setupAccelDataView(self):
        # reconfigure the right pannel for data show
        #for child in self.right_frame.winfo_children():
        #    child.destroy()        
        
        #self.fig       = Figure(figsize=(5, 4), dpi=100)
        self.ax.clear()
        self.x_data    = np.arange(0, 3, .1)
        self.y_data    = np.zeros((len(self.x_data),3))
        #self.ax        = self.fig.add_subplot()
        self.ax.plot(self.x_data, self.y_data)
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("Accel(t)")
        self.ax.set_ylim(-1500, 1500)
        self.ax.legend(['X', 'Y', 'Z'])

        #self.canvas= FigureCanvasTkAgg(self.fig, master=self.right_frame)
        #self.canvas.get_tk_widget().grid(row=1,column=0) #,rowspan = 4, sticky="NSEW") #,columnspan = 4,rowspan = 4)
        #self.canvas.draw()
            
    # -- Task --            
    def acquireAccelRealTime(self, k=0):
        # Update subplots 
        self.time_start = time.time()
        while self.btnAcquireStatus:
            #self.tprint('%s' %k)
            # Read temperature
            sens_val    = self.getAccelValues()
            time_now    = time.time() - self.time_start

            # # Add x and y to lists
            # self.x_data.append(time_now)
            # self.y_data.append(sens_val[0:3]) # drop temperature
            #xdata       = self.line.get_xdata()
            

            # Limit x and y lists to the more recent items
            size_limit = 30
            # self.x_data = self.x_data[-size_limit:]
            # self.y_data = self.y_data[-size_limit:]  
            xdata       = self.ax.lines[0].get_xdata()
            xdata[0:-1] = xdata[1:]
            xdata[-1]   = time_now           
            
            for k in range(3):

                ydata       = self.ax.lines[k].get_ydata()
                ydata[0:-1] = ydata[1:]
                ydata[-1]   = np.array(sens_val[k])
                self.ax.lines[k].set_ydata(ydata)
                self.ax.lines[k].set_xdata(xdata)
                  
            # scale plot in x values
            self.ax.set_xlim([xdata[0],xdata[-1]])
            
            #self.line.set_data(np.array(self.x_data), np.array(self.y_data))
            #self.line.set_data(xdata, ydata)

            # required to update canvas and attached toolbar!
            self.canvas.draw()                         
            time.sleep(0.01)  
      
    # -- Config --      
    def btnAccelPress(self):
        # when pressed starts or stops thread
        global stopThread
        if self.btnAcquireStatus == False:
            self.setupAccelDataView()
            stopThread = False
            self.btnAcquireStatus = True
            self.btnAccel.config(bg="red", fg="white", text="Stop")
            
            self.ts = Thread(target=self.acquireAccelRealTime)
            #self.ts.daemon = True
            self.ts.start()  
            self.tprint('Thread is running')   
            
        else:
            self.btnAcquireStatus = False
            stopThread = True
            self.btnAccel.config(bg="gray", fg="black", text="Get Accel")
            #self.ts.join()
            self.tprint('Thread is stopped')  
            
    # ---------------------------------   
    # -- Configure -- 
    def setupEcgRawDataView(self):
        # reconfigure the right pannel for data show
        #for child in self.right_frame.winfo_children():
        #    child.destroy()        
        
        #self.fig       = Figure(figsize=(5, 4), dpi=100)
        self.ax.clear()
        self.x_data    = np.arange(0, 3, .1)
        self.y_data    = np.zeros((len(self.x_data),1))
        #self.ax        = self.fig.add_subplot()
        self.ax.plot(self.x_data, self.y_data)
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("ECG(t)")
        self.ax.set_ylim(-250, 250)
        self.ax.legend(['ECG'])

        #self.canvas= FigureCanvasTkAgg(self.fig, master=self.right_frame)
        #self.canvas.get_tk_widget().grid(row=1,column=0) #,rowspan = 4, sticky="NSEW") #,columnspan = 4,rowspan = 4)
        #self.canvas.draw()
                                 
    # -- Task --
    def acquireEcgRawRealTime(self, k=0):
        # Update subplots 
        self.time_start = time.time()
        while self.btnAcquireStatus:
            #self.tprint('%s' %k)
            # Read temperature
            sens_val    = self.getEcgRawValues()
            time_now    = time.time() - self.time_start

            # # Add x and y to lists
            # self.x_data.append(time_now)
            # self.y_data.append(sens_val[0:3]) # drop temperature
            #xdata       = self.line.get_xdata()
            

            # Limit x and y lists to the more recent items
            size_limit = 30
            # self.x_data = self.x_data[-size_limit:]
            # self.y_data = self.y_data[-size_limit:]  
            xdata       = self.ax.lines[0].get_xdata()
            xdata[0:-1] = xdata[1:]
            xdata[-1]   = time_now           

            ydata       = self.ax.lines[0].get_ydata()
            ydata[0:-1] = ydata[1:]
            ydata[-1]   = np.array(sens_val)
            self.ax.lines[0].set_ydata(ydata)
            self.ax.lines[0].set_xdata(xdata)
                  
            # scale plot in x values
            self.ax.set_xlim([xdata[0],xdata[-1]])
            
            #self.line.set_data(np.array(self.x_data), np.array(self.y_data))
            #self.line.set_data(xdata, ydata)

            # required to update canvas and attached toolbar!
            self.canvas.draw()                         
            time.sleep(0.01) 
            
    # -- Control --
    def btnEcgRawPress(self):
        # when pressed starts or stops thread
        global stopThread
        if self.btnAcquireStatus == False:
            self.setupEcgRawDataView()
            self.btnAcquireStatus = True
            stopThread = False
            self.btnEcgRaw.config(bg="red", fg="white", text="Stop")
            
            self.ts = Thread(target=self.acquireEcgRawRealTime)
            #self.ts.daemon = True
            self.ts.start()  
            self.tprint('Thread is running')   
            
        else:
            self.btnAcquireStatus = False
            stopThread = True
            self.btnEcgRaw.config(bg="gray", fg="black", text="Get ECG")
            #self.ts.join()
            self.tprint('Thread is stopped')             

   # ---READ DATA FROM FILE ---------------------------------------
   # -- Setup  --
    def setupFileOffLine(self):
        # read csv file raw by raw
        import csv

        csv_path = r'C:\RobotAI\Customers\Levron\Data\2022-12-10\Event_20_2022_11_15_15_18_00.642.csv';
        try:
            csvfile = open(csv_path)
        except Exception as e:
            print(e)
            return
        self.reader = csv.DictReader(csvfile)
        # with open(csv_path, newline='') as csvfile:
        #     self.reader = csv.DictReader(csvfile)       
        
        self.ax.clear()
        self.x_data    = np.arange(0, 5, .01)
        self.y_data    = np.zeros((len(self.x_data),3))
        #self.ax        = self.fig.add_subplot()
        self.ax.plot(self.x_data, self.y_data)
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("f(t)")
        self.ax.set_ylim(-150, 250)
        self.ax.legend(['LV','Surface','PleuralPressure'])     

    # -- Task --
    def processFileOffLine(self, k=0):
        # Update subplots 
        global stopThread
        self.time_start = time.time()
        cnt = 0
        for row in self.reader:
            
            cnt = cnt + 1
            
            # decimate
            if np.mod(cnt,10) != 0:
                continue
            
            #time_s = time.time()
            # user stop
            if stopThread: #not self.btnAcquireStatus:
                self.tprint('Stop is detected')
                break
            
            # Read time and values
            time_now = float(row['Time'].split()[0])
            lv_value = float(row['LV'])
            surf_value = float(row['Surface'])
            press_value = float(row['PleuralPressure'])
            
            #self.tprint('Time decode %f' %(time.time()-time_s))

            # Limit x and y lists to the more recent items
            #size_limit = 300
            # self.x_data = self.x_data[-size_limit:]
            # self.y_data = self.y_data[-size_limit:]  
            xdata       = self.ax.lines[0].get_xdata()
            xdata[0:-1] = xdata[1:]
            xdata[-1]   = time_now           

            sens_val   = [lv_value,surf_value, press_value]
            for k in range(3):

                ydata       = self.ax.lines[k].get_ydata()
                ydata[0:-1] = ydata[1:]
                ydata[-1]   = np.array(sens_val[k])
                self.ax.lines[k].set_ydata(ydata)
                self.ax.lines[k].set_xdata(xdata)

           # scale plot in x values
            self.ax.set_xlim([xdata[0],xdata[-1]])
            
            #self.tprint('Time set lines %f' %(time.time()-time_s))
            
            # required to update canvas and attached toolbar!
            self.canvas.draw()                         
            #time.sleep(0.01) 
            #self.tprint('Total cycle %f' %(time.time()-time_s))

    # -- Control --        
    def btnFileOffLinePress(self):
        # when pressed starts or stops thread
        global stopThread
        if self.btnAcquireStatus == False:
            self.setupFileOffLine()
            self.btnAcquireStatus = True
            stopThread = False
            self.btnFileOffLine.config(bg="red", fg="white", text="Stop")
            
            self.ts = Thread(target=self.processFileOffLine)
            #self.ts.daemon = True
            self.ts.start()  
            self.tprint('Thread is running')   
            
        else:
            self.btnAcquireStatus = False
            stopThread = True
            self.btnFileOffLine.config(bg="white", fg="black", text="Process File")
            #self.tprint('Thread is sopping')
            #self.ts.join()
            self.tprint('Thread is stopped')                         

   # ---READ DATA FROM FILE AND PROCESS in ARDUINO----------
   
   # -- Setup --
    def setupFileOnLine(self):
        # read csv file raw by raw
        import csv

        csv_path = r'C:\RobotAI\Customers\Levron\Data\2022-12-10\Event_20_2022_11_15_15_18_00.642.csv';
        try:
            csvfile = open(csv_path)
        except Exception as e:
            print(e)
            return
        self.reader = csv.DictReader(csvfile)
        # with open(csv_path, newline='') as csvfile:
        #     self.reader = csv.DictReader(csvfile)       
        
        self.ax.clear()
        self.x_data    = np.arange(0, 5, .01)
        self.y_data    = np.zeros((len(self.x_data),4))
        #self.ax        = self.fig.add_subplot()
        self.ax.plot(self.x_data, self.y_data)
        self.ax.set_xlabel("time [s]")
        self.ax.set_ylabel("f(t)")
        self.ax.set_ylim(-50, 50)
        self.ax.legend(['LV','Surface','PleuralPressure','Algo'])     

    # -- Task --
    def processFileOnLine(self, k=0):
        # Update subplots 
        global stopThread
        self.time_start = time.time()
        cnt = 0
        for row in self.reader:
            
            if stopThread: #not self.btnAcquireStatus:
                self.tprint('Stop is detected')
                break            

            # decimate
            cnt = cnt + 1
            if np.mod(cnt,10) != 0:
                continue
            
            # Read time and values
            time_now = float(row['Time'].split()[0])
            lv_value = float(row['LV'])
            surf_value = float(row['Surface'])
            press_value = float(row['PleuralPressure'])
            
            # send receive
            alg_value = self.setFileData(valLV = lv_value, valSurf = surf_value, valPress = press_value)
            
            #self.tprint('Time decode %f' %(time.time()-time_s))

            # Limit x and y lists to the more recent items
            #size_limit = 300
            # self.x_data = self.x_data[-size_limit:]
            # self.y_data = self.y_data[-size_limit:]  
            xdata       = self.ax.lines[0].get_xdata()
            xdata[0:-1] = xdata[1:]
            xdata[-1]   = time_now           

            sens_val   = [lv_value,surf_value, press_value, alg_value]
            for k in range(4):

                ydata       = self.ax.lines[k].get_ydata()
                ydata[0:-1] = ydata[1:]
                ydata[-1]   = np.array(sens_val[k])
                self.ax.lines[k].set_ydata(ydata)
                self.ax.lines[k].set_xdata(xdata)

           # scale plot in x values
            self.ax.set_xlim([xdata[0],xdata[-1]])
            
            #self.tprint('Time set lines %f' %(time.time()-time_s))
            
            # required to update canvas and attached toolbar!
            self.canvas.draw()                         
            #time.sleep(0.01) 
            #self.tprint('Total cycle %f' %(time.time()-time_s))

    # -- Control --        
    def btnFileOnLinePress(self):
        # when pressed starts or stops thread
        global stopThread
        if self.btnAcquireStatus == False:
            self.setupFileOnLine()
            #self.btnAcquireStatus = True
            stopThread = False
            self.btnFileOnLine.config(bg="red", fg="white", text="Stop")
            
            self.ts = Thread(target=self.processFileOnLine)
            #self.ts.daemon = True
            self.ts.start()  
            self.tprint('Thread is running')   
            
        else:
            self.btnAcquireStatus = False
            stopThread = True
            self.btnFileOnLine.config(bg="white", fg="black", text="File on Board")
            #self.tprint('Thread is sopping')
            #self.ts.join()
            self.tprint('Thread is stopped')                         
      
    # ------------------------------------------

    def btnA(self):
        global ledAstatus, ledBstatus, servoPos

        if ledAstatus == 0:
            ledAstatus = 1
            self.ledAbutton.config(bg="white", fg="black")
            
        else:
            ledAstatus = 0
            self.ledAbutton.config(fg="white", bg="black")

        self.robotGripperOnOff(ledAstatus)

    def btnB(self):
        global ledAstatus, ledBstatus, servoPos

        if ledBstatus == 0:
            ledBstatus = 1
            self.ledBbutton.config(bg="white", fg="black")
        else:
            ledBstatus = 0
            self.ledBbutton.config(fg="white", bg="black")
        #self.com.valToArduino(ledAstatus, ledBstatus, servoPos)

        self.robotGripperOnOff(ledBstatus)
        

   # ---COMM IF----------
   
    def sliderUpdate(self,sval):
        # update slider

        self.sliderCount = int(sval)
        #self.com.valToArduino(ledAstatus, ledBstatus, servoPos)

        #msgSend   = Message(1,[ledAstatus, ledBstatus, servoPos])
        #msgRecv   = Message(2,[0, 0, 0])
        #self.com.msgSendRecv(msgSend,msgRecv)

           
        

    def tprint(self, txt='',level='I', showInCmdLine = True):
        try:
            self.status.delete(0,tk.END)
        except:
            print('Main window is destroyed')
            return

        if level == 'I':
            ptxt = 'I: GUI: %s' % txt
            bckg = "White"
            log.info(ptxt)
        if level == 'W':
            ptxt = 'W: GUI: %s' % txt
            bckg = "Yellow"
            log.warning(ptxt)
        if level == 'E':
            ptxt = 'E: GUI: %s' % txt
            bckg = "Red"
            log.error(ptxt)
            
        #print(ptxt)    

        if showInCmdLine:
            self.status.insert(0,txt)
            self.status.config({"background": bckg})




#%% Main Code


def MainGUI(version = '0000', module_dict = {}):
    #%
    global gui_version
    gui_version = version
    
    # logging.basicConfig(handlers=[RotatingFileHandler('./file.log', maxBytes=50000000, backupCount=10)],
    #                     level=logging.INFO, format="%(asctime)s %(levelname)-8s %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    # # log a message
    # logging.info('====== Starting up the program %s =======' % version)
    # do not show the logging on the console
    #logging.propagate = False
    
    # Logger

    #import sys
    
    log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")

    

    # # key board short cuts
    # def keyPressedCallback(event):
    #     #win.focus_set()
    #     # The obvious information
    #     c = event.keysym
    #     s = event.state
    #     shift = (s & 0x1) != 0
    #     # protect
    #     if not shift:
    #         return
    #     if event.char == 'a':
    #         app.CameraImageSave()
    #     if event.char == 'r':
    #         app.RobotCameraImageSave()
    #     if event.char == 'q':
    #         app.stop = True
    #     #print("pressed", repr(event.char))




    #cfg         = module_dict['cfg']
    # prj         = module_dict['prj'] # load session
    # cam         = module_dict['cam']
    # obj         = module_dict['obj']
    # rob         = module_dict['rob']
    # lbl         = module_dict['lbl']


#    prj    = ProjectManager(config = cfg) # load session
#    cam    = CameraManager(config = cfg)
#    obj    = ObjectManager(config = cfg)
#    rob    = RobotManager(config = cfg)
#    lbl    = LabelManager(config = cfg)




    #app         = RobotAIGUI(version, win, cam, prj, obj, rob, lbl) # object instantiated
    # check if requested
    #app.AutoStart()


    #win.mainloop()
    #app.Finish()


    win = tk.Tk()
    #win = ctk.CTk()
    #win.iconbitmap(iconFilePath)
    w = win.winfo_screenwidth()
    h = win.winfo_screenheight()
    #win.geometry("%dx%d+100+100" % (w/2, h/2))
    win.geometry("1200x800+100+100")

    gui = MonitorGUI(win, module_dict)
    gui.setupView()
    #win.bind("<Key>", gui.KeyPressedCallback)
    #atexit.register(gui.finish())
    win.protocol("WM_DELETE_WINDOW", gui.finish)    
    win.mainloop()
    
    #gui.finish()
    log.shutdown()
# --------------------------
if __name__ == '__main__':
    MainGUI()
