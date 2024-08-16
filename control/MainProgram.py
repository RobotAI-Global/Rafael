# -*- coding: utf-8 -*-
"""

@author: avita

RobotAI : Project Manager - Main Task
Usage :


Install:
    Zion -> env :  D:/RobotAI/Design/env/inspect6d/python.exe : matplotlib 3.0.3

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101    11.08.24 UD     Created
-----------------------------

"""
server_version   = '0101'

#import os
#import tkinter as tk
#import threading
#import json
import logging as log
import time   # just to measure switch time
#import logging


#base_path     = os.path.abspath(".")
#base_path      = r'D:\RobotAI\Design\apps\PickManager\gui\logo.ico' #os.path.dirname(os.path.realpath(__file__))
#iconFilePath  =  '.\\gui\\logo.ico' #os.path.join(base_path, '\logo.ico')
#base_path      = os.path.abspath(".")
#iconFilePath  =  r'C:\robotai\SW\RobotManager\gui\logo.ico' #os.path.join(base_path, '.\\gui\\logo.ico')

#import os
#import sys
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\robot')
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\vision')

#import os

log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")


#import numpy as np
#from threading import Thread


#try:
from control.ConfigManager import ConfigManager
from robot.Robot import Robot as RobotManager
from host.ComServer  import ComServer as HostManager
#from control.StateMachine import StateMachine
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

#%% 
# StateMachine/State.py
# A State has an operation, and can be moved into the next State given an Input:
from enum import Enum
class STATE(Enum):
    INIT                = 1    
    
    WAIT_FOR_COMMAND    = 20
    EXECUTE             = 10
    SPECIAL             = 90     # deal with special message
    FINISH              = 100
    ERROR               = 401
    
# A State has an operation, and can be moved into the next State given an Input:
class ERROR(Enum):
    NONE                = 0    
    NO_CONNECTION       = 101
    NO_HOME_POSITION    = 102     # deal with special message
    FINISH      = 100
    ERROR       = 401


#%% 
class MainProgram:

    def __init__(self, version = "0001", module_dict = {}):

        self.cfg        = ConfigManager()
        self.debugOn    = False
        self.ts         = None # task handle
        self.state      = STATE.INIT
        self.error      = ERROR.NONE
        
        # main state machine
        #self.rsm        = StateMachine(parent = self)
        
        # robot comm
        self.rbm        = RobotManager(parent = self)

        # host comm
        self.host       = HostManager(parent = self)    
        
        # connectio to IO
        self.ioc        = IOController(parent = self)
        

    ## -------------------------------
    #  -- Init All---
    ## -------------------------------
    def Init(self):
        "intialize all the modules - read some inint data from config file"
        self.cfg.Init()
        self.rbm.Init()
        self.ioc.Init()
        self.host.Init()
        #self.rsm.Init()
        self.state = STATE.INIT
        self.error = ERROR.NONE
        self.Print('Init')
        
    def Start(self):
        "start running"        
        self.host.Start()
        self.rbm.Start()
        self.ioc.Start()
        self.Print('Start')
        
    ## -------------------------------
    #  -- ACTIONS ---
    ## -------------------------------   
    def ActionHome(self):
        "set system in home position"
        ret         = False
        return ret
    
    def ActionStop(self):
        "check if"
        ret         = False
        return ret    
    def ActionLoadUUTToTable(self):
        "check if"
        ret         = False
        return ret
    
    def ActionUnloadUUTFromTable(self):
        "check if"
        ret         = False
        return ret
    
    def ActionLoadUUTToTestStand(self):
        "check if"
        ret         = False
        return ret
    
    def ActionUnloadUUTFromTestStand(self):
        "check if"
        ret         = False
        return ret
    
    ## -------------------------------
    #  -- Conditions ---
    ## -------------------------------   
    def CheckConnection(self):
        "check if all modules are connected"
        ret         = True
        ret         = self.rbm.IsConnected() and ret
        ret         = self.host.IsConnected() and ret
        ret         = self.ioc.IsConnected() and ret
        self.Print('Connectivity : %b' %ret)
        return ret   
    
    def CheckSystemState(self):
        "check if all modules are in home position"
        ret         = True
        ret         = self.rbm.IsHome() and ret
        ret         = self.host.IsHome() and ret
        ret         = self.ioc.IsHome() and ret
        self.Print('Home position : %b' %ret)
        return ret       
    
    def WaitFor(self):
        "check if"
        ret         = False
        return ret
    
    def WaitForStop(self):
        "check if"
        ret         = False
        return ret
    
    def WaitForLoadUUTToTable(self):
        "check if"
        ret         = False
        return ret
    
    def WaitForUnloadUUTFromTable(self):
        "check if"
        ret         = False
        return ret
    
    def WaitForLoadUUTToTestStand(self):
        "check if"
        ret         = False
        return ret
    
    def WaitForUnloadUUTFromTestStand(self):
        "check if"
        ret         = False
        return ret    
        
    ## -------------------------------
    #  -- STATES ---
    ## -------------------------------                
    def StateSpecialMessage(self, msg_in, curr_state):
        "special messages"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state
    
    def StateInit(self, msg_in, curr_state):
        "do nonthing"
        msg_out     = msg_in
        next_state  = curr_state
        
        # do we have conection
        ret         = self.CheckConnection()
        if not ret:
            self.error = ERROR.NO_CONNECTION
            next_state = STATE.ERROR
            
        # do we have initial position
        ret         = self.CheckSystemState()
        if not ret:
            self.error = ERROR.NO_HOME_POSITION
            next_state = STATE.ERROR            
        
        return msg_out, next_state 
    
    def StateWaitForCommand(self, msg_in, curr_state):
        "do onthing"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state     
    
    def StateFinish(self, msg_in, curr_state):
        "do onthing"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state 

    def StateError(self, msg_in, curr_state):
        "do onthing"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state  

    def StateExecute(self, msg_in, curr_state):
        "do everything"
        msg_out     = msg_in
        next_state  = curr_state


#        if not isinstance(msgPacket, Packet):
#            print('Wrong object')         
    
        if msg_in.command == 0:
            self.Print('0 - DEFULT command')           
            self.ActionHome()
            
        if msg_in.command == 1:
            self.Print('1 - STOP command')
            self.ActionStop()
            
        if msg_in.command == 2:
            self.Print('2 - Load UUT to index table command')
            self.ActionLoadUUTToTable()
            # self.r.move_joint(target_joint=[0,90,0,90,0,0],speed=100,accelearation=70)
            # self.r.move_joint(target_joint=[0,0,90,0,0,0],speed=100,accelearation=70)
            
        if msg_in.command == 3:
            self.Print('3 - Unload UUT from index table command')
            self.ActionUnloadUUTFromTable()
            
        if msg_in.command == 4:
            self.Print('4 - Load UUT to test stand command')
            self.ActionLoadUUTToTestStand()
            
        if msg_in.command == 5:
            self.Print('5 - Unload UUT from test stand command')
            self.ActionUnloadUUTFromTestStand()
            
        if msg_in.command == 6:
            self.Print('6 - Get staus command')
            msg_out.msgSize = 60
            msg_out.cmdCode = 53
            
            msg_out.last_cmd_status = self.msgPacketInternal.last_cmd_status
            msg_out.general_status[0] = 1
            msg_out.general_status[1] = 2
            msg_out.general_status[2] = 3
            msg_out.general_status[3] = 4
            msg_out.general_status[4] = 5
            msg_out.general_status[5] = 6
            msg_out.general_status[6] = 7
            msg_out.general_status[7] = 8
            msg_out.general_status[8] = 9
            msg_out.general_status[9] = 10
            
            # Preparing data to be send to client 
            # Sending packet only when commands are 6 or 7 
            # Other commands just execute robot movments and logic
          
            
        if msg_in.command == 7:
            self.Print('7 - Get bit results command')
            msg_out.msgSize         = 64
            msg_out.cmdCode         = 52                
            
            # msgPacket.bit_status     = self.msgPacketInternal.bit_status
            msg_out.bit_status      = 1
            msg_out.seconds         = 45
            msg_out.error_codes[0] = 1
            msg_out.error_codes[1] = 2
            msg_out.error_codes[2] = 3
            msg_out.error_codes[3] = 4
            msg_out.error_codes[4] = self.msgPacketInternal.error_codes[4]
            msg_out.error_codes[5] = 6
            msg_out.error_codes[6] = 7
            msg_out.error_codes[7] = 8
            msg_out.error_codes[8] = 9
            msg_out.error_codes[9] = 10 
            
        if msg_in.command == 8:
            self.Print('8 - Connections counter zeroise command')             
    
        return msg_out, next_state  
    
        
    def Transition(self, msg_in):
        "transition to a different state"
        curr_state = self.state
        next_state = self.state

        
        # deal with special messages
        msg_out, curr_state = self.StateSpecialMessage(msg_in, curr_state)
        
        # deal with messages per state
        if curr_state == STATE.INIT:
            msg_out, next_state = self.StateInit(msg_in, curr_state)
        elif curr_state == STATE.WAIT_FOR_COMMAND:
            msg_out, next_state = self.StateWaitForCommand(msg_in, curr_state)            
            
        elif curr_state == STATE.EXECUTE:
            msg_out, next_state = self.StateExecute(msg_in, curr_state) 
        elif curr_state == STATE.SPECIAL:
            msg_out, next_state = self.StateExecute(msg_in, curr_state)             
        elif curr_state == STATE.FINISH:
            msg_out, next_state = self.StateFinish(msg_in, curr_state)
        elif curr_state == STATE.ERROR:
            msg_out, next_state = self.StateError(msg_in, curr_state)            
        else:
            msg_out, next_state = msg_in, STATE.ERROR
            self.Print('Not supprted state')
            
        
        self.Print('Transition to %s' %str(next_state))   
        self.state = next_state
        return msg_out        
        
        
    ## -------------------------------
    #  -- TASK ---
    ## -------------------------------
    def MainTask(self):
        "run all modules"
        isStop = False
        while not isStop:
            
            # receive message from the host
            msgRx = self.host.RecvMessage()
            
            # send message to the state machine
            msgTx  = self.Transition(msgRx)
            
            # send response to the host
            isOk   = self.host.SendMessage(msgTx)


    def Print(self, txt='',level='I'):

        if level == 'I':
            ptxt = 'I: PRG: %s' % txt
            #log.info(ptxt)
        if level == 'W':
            ptxt = 'W: PRG: %s' % txt
            #log.warning(ptxt)
        if level == 'E':
            ptxt = 'E: PRG: %s' % txt
            #log.error(ptxt)
        print(ptxt)


            
#%% Testing - unittest
class TestMainProgram:
    def __init__(self):
        self.mp = MainProgram()
        
    def test_current_state(self):
        self.mp.Print('1')
        assert self.mp.state == STATE.INIT
        
    def test_init(self):
        self.mp.Init()
        assert self.mp.state == STATE.INIT        
        
    def test_start(self):
        self.mp.Init()
        self.mp.Start()
        assert self.mp.state == STATE.INIT  
        

# --------------------------
if __name__ == '__main__':
    #from MainProgram import MainProgram
    tst = TestMainProgram()
    #tst.test_current_state() # ok
    #tst.test_init() # ok 
    tst.test_start() # ok 