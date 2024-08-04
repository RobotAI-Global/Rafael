# -*- coding: utf-8 -*-
"""

@author: avita

RobotAI : Robot Manager - Main Server
Usage :


Install:
    Zion -> env :  D:/RobotAI/Design/env/inspect6d/python.exe : matplotlib 3.0.3

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101    11.06.24 UD     Created
-----------------------------

"""
server_version   = '0101'

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
#import sys
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\robot')
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\vision')

#import os

log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")

    
#sys.path.insert(1, r'.\\vision')


#import tkinter as tk
#import tkinter.ttk as ttk
#import customtkinter as ctk
#from tkinter import *

#from MonitorMessage import Message, MESSAGE_TYPE
#import atexit

# def exit_handler():
#     print('My application is ending!')
#     closeSerial()
# atexit.register(exit_handler)


#import matplotlib
#matplotlib.use('TkAgg')
import numpy as np
from threading import Thread


#import matplotlib as mpl
#mpl.rcParams['toolbar'] = 'None' 


#try:
from gui.ConfigManager import ConfigManager
from robot.Robot import Robot as RobotManager
from host.ComServer  import ComServer as HostManager
from control.StateMachine import StateMachine
from disc.ControllerHHC import IOController

#except Exception as e:
#    from ConfigManager import ConfigManager
#    from DisplayManager  import DisplayManager
#    from RobotAPI import RobotAPI as RobotManager    
#    # debug
#    print(e)
#    print('Load them manually and then switch directory')
#    #from MsgJsonSimple import MsgPoseData
#




#
#AXIS_VIEW_OPTIONS             = ["45-45","Left","Top"] #et
#SAFETY_MARGIN_Z               = 50  # mm
#TX_STEP_SIZE                  = 0.0400 # metr
#SCAN_POINTS                   = [[-500,-200, 400, 94, 0, -82],[-500,0,400, 94, 0, -82],[-500,0,600, 94, 0, -82],[-500,-220,600, 94, 0, -82],[-665.01 ,-251.23 , 648.96 , 94, 0, -82]]
HOME_POSE                     = [-500.0, 38.0, 430.0, 94.3088530785335, 0.6875493541569879, -82.21944360127314]
#        
#WORK_POINTS  = [[-0.520,-0.200, 0.400, -1.64,0, 1.7],   [-0.750,-0.200, 0.400, -1.64,0, 1.7],
#                [-0.500, 0,     0.400, -1.64,0, 1.7],   [-0.750, 0,     0.400, -1.64,0, 1.7],
#                [-0.500, 0,     0.600, -1.64,0, 1.7],   [-0.650, 0,     0.600, -1.64,0, 1.7],
#                [-0.500,-0.220, 0.600, -1.64,0, 1.7],   [-0.750,-0.220, 0.600, -1.64,0, 1.7]]




#%% 
class MainProgram:

    def __init__(self, version = "0001", module_dict = {}):

        self.cfg        = ConfigManager()
        self.debugOn    = False
        self.ts         = None # task handle
        
        # main state machine
        self.rsm        = StateMachine(parent = self)
        
        # robot comm
        self.rbm        = RobotManager(parent = self)

        # host comm
        self.host       = HostManager(parent = self)    
        
        # connectio to IO
        self.ioc        = IOController(parent = self)
        
        #self.ip_robot  = "192.168.1.16"
        #self.port_robot = 5033
        #self.rbm        = None  # robot manager
         #ip = self.ip_robot, port = self.port_robot) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)




        self.robot_speed = 100  # some strnage numbers
        self.robot_pose  = np.zeros((1,6))
        #self.home_pose   = np.array([-86.2, -225.6, 237.6, -177.0, 2.29, 138.6]) # chess
        self.home_pose   = np.array(HOME_POSE)
        
        # receive message
        #self.msgRecv   = {'Id': 0, 'Data' : 0}
        
        # host
        #self.ip_vision         = '127.0.0.1'    
        #self.ip_vision         = '192.168.1.130' # Uri comp
        #self.port_vision       = 5555
        #self.vis        = None #VisionManager(self.cfg)        
        #self.sim        = None #MonitorComm(cfg)        
        #self.sim_ts     = None  # simulator task handle
        self.object_pose  = np.zeros((1,6))
        
        
    ## -------------------------------
    #  -- Init All---
    ## -------------------------------
    def initModules(self):
        "intialize all the modules - read some inint data from config file"
        self.cfg.init()
        self.rbm.init()
        self.ioc.init()
        self.host.Init()
        self.rsm.init()
        
    def startModules(self):
        "start running"
        
        self.host.RunThread()
        
        
    ## -------------------------------
    #  -- TASK ---
    ## -------------------------------
    def mainTask(self):
        "run all modules"
        isStop = False
        while not isStop:
            
            # receive message from the host
            msgRx = self.host.recv()
            
            # send message to the state machine
            msgTx  = self.rsm.transition(msgRx)
            
            # send response to the host
            isOk   = self.host.send(msgTx)


        
    ## -------------------------------
    #  -- Decode Command ---
    ## -------------------------------    
    def ExecutePacket(self):
        
        if self.queueToRobot.empty():
            return
        
        msgPacket  = self.queueToRobot.get()
        
        if not isinstance(msgPacket, Packet):
            print('Wrong object')         
    
        if msgPacket.command == 0:
            print('0 - DEFULT command')           
            self.Home()
            
        if msgPacket.command == 1:
            print('1 - STOP command')
            self.Stop()
            
        if msgPacket.command == 2:
            print('2 - Load UUT to index table command')
            self.LoadUUTToTable()
            # self.r.move_joint(target_joint=[0,90,0,90,0,0],speed=100,accelearation=70)
            # self.r.move_joint(target_joint=[0,0,90,0,0,0],speed=100,accelearation=70)
            
        if msgPacket.command == 3:
            print('3 - Unload UUT from index table command')
            self.UnloadUUTFromTable()
            
        if msgPacket.command == 4:
            print('4 - Load UUT to test stand command')
            self.LoadUUTToTestStand()
            
        if msgPacket.command == 5:
            print('5 - Unload UUT from test stand command')
            self.UnloadUUTFromTestStand()
            
        if msgPacket.command == 6:
            print('6 - Get staus command')
            msgPacket.msgSize = 60
            msgPacket.cmdCode = 53
            
            msgPacket.last_cmd_status = self.msgPacketInternal.last_cmd_status
            msgPacket.general_status[0] = 1
            msgPacket.general_status[1] = 2
            msgPacket.general_status[2] = 3
            msgPacket.general_status[3] = 4
            msgPacket.general_status[4] = 5
            msgPacket.general_status[5] = 6
            msgPacket.general_status[6] = 7
            msgPacket.general_status[7] = 8
            msgPacket.general_status[8] = 9
            msgPacket.general_status[9] = 10
            
            # Preparing data to be send to client 
            # Sending packet only when commands are 6 or 7 
            # Other commands just execute robot movments and logic
            self.queueFromRobot.put(msgPacket)            
            
        if msgPacket.command == 7:
            print('7 - Get bit results command')
            msgPacket.msgSize = 64
            msgPacket.cmdCode = 52                
            
            # msgPacket.bit_status     = self.msgPacketInternal.bit_status
            msgPacket.bit_status     = 1
            msgPacket.seconds = 45
            msgPacket.error_codes[0] = 1
            msgPacket.error_codes[1] = 2
            msgPacket.error_codes[2] = 3
            msgPacket.error_codes[3] = 4
            msgPacket.error_codes[4] = self.msgPacketInternal.error_codes[4]
            msgPacket.error_codes[5] = 6
            msgPacket.error_codes[6] = 7
            msgPacket.error_codes[7] = 8
            msgPacket.error_codes[8] = 9
            msgPacket.error_codes[9] = 10 
            
            # Preparing data to be send to client
            # Sending packet only when commands are 6 or 7 
            # Other commands just execute robot movments and logic
            self.queueFromRobot.put(msgPacket)
            
        if msgPacket.command == 8:
            print('8 - Connections counter zeroise command')             
    
    ## -----------------------------    
    # -- Session Control ---
    ## -----------------------------    
        
    def getVersion(self):
        # start
        self.tprint('Reading version  ...')
        
        # maybe already running
        self.tprint('App version : %s' %str(server_version))
        
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
        #JsonEditor(fname)
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
    
 
    def startAuto(self):
        # start auto connections
        #self.robotConnect()
        #self.robotStatus()
        #self.hostConnect()
        #self.hostStatus()
        pass

       

    def tprint(self, txt='',level='I'):

        if level == 'I':
            ptxt = 'I: SRV: %s' % txt
            log.info(ptxt)
        if level == 'W':
            ptxt = 'W: SRV: %s' % txt
            log.warning(ptxt)
        if level == 'E':
            ptxt = 'E: SRV: %s' % txt
            log.error(ptxt)



            
#%% Testing - unittest
class TestMainProgram:
    def __init__(self):
        self.ms = MainProgram()
        
    def test_current_state(self):
        self.ms.tprint('1')
        assert self.ms.currentState == STATE.INIT
        
    def test_all(self):
        self.test_current_state()
        

# --------------------------
if __name__ == '__main__':
    MainGUI()
