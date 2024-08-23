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
#import logging as log
#import time   # just to measure switch time
#import logging
from threading import Thread

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
#%% Logger
import logging
logger      = logging.getLogger("robot")
formatter   = logging.Formatter('[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s', datefmt="%M:%S", style="{")
logger.setLevel("DEBUG")

console_handler = logging.StreamHandler()
console_handler.setLevel("DEBUG")
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

file_handler = logging.FileHandler("main_app.log", mode="a", encoding="utf-8")
file_handler.setLevel("WARNING")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

#log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")


#import numpy as np
#from threading import Thread

#%% 
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
    HOME                = 2    
    
    WAIT_FOR_COMMAND    = 20    # host command
    LOAD_UUT_TO_TABLE   = 30    # host command 
    UNLOAD_UUT_FROM_TABE= 40
    LOAD_UUT_TO_STAND   = 50
    UNLOAD_UUT_FROM_STAND = 60  
    GET_AND_SEND_STATUS = 70     # send status to host
    GET_AND_SEND_BIT_RESULTS = 71
    SPECIAL             = 80     # deal with special message
    STOP                = 90     # emergency stop
    FINISH              = 100
    ERROR               = 401
    
# A State has an operation, and can be moved into the next State given an Input:
class ERROR(Enum):
    NONE                = 0    
    NO_CONNECTION       = 101
    NO_HOME_POSITION    = 102     # deal with special message
    CAN_NOT_STOP        = 103
    BAD_HOST_COMMAND    = 104
    TESTER_OPEN_DOOR    = 201
    TESTER_OPEN_BUHNA   = 202
    FINISH              = 300
    ERROR               = 401


#%% 
class MainProgram:

    def __init__(self, version = "0001", module_dict = {}):

        self.cfg        = ConfigManager()
        self.debugOn    = False
        self.ts         = None # task handle
        self.state      = STATE.INIT
        self.error      = ERROR.NONE
        self.stop       = False
        
        # main state machine
        #self.rsm        = StateMachine(parent = self)
        
        # robot comm
        self.rbm        = RobotManager(parent = self)

        # host comm
        self.host       = HostManager(parent = self)    
        
        # connectio to IO
        self.ioc        = IOController(parent = self)
        
        self.Print('Created')
        

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
        return True
        
    def Stop(self):
        "stop everything"        
        self.host.Stop()
        self.rbm.Stop()
        self.ioc.Stop()
        self.Print('Stop')   
        return True
        
    ## -------------------------------
    #  -- Conditions ---
    ## -------------------------------   
    def CheckConnection(self):
        "check if all modules are connected"
        ret         = True
        ret         = self.rbm.IsConnected() and ret
        ret         = self.host.IsConnected() and ret
        ret         = self.ioc.IsConnected() and ret
        self.Print('Connectivity : %s' %str(ret))
        return ret   
    
    def CheckSystemState(self):
        "check if all modules are in home position"
        ret         = True
        ret         = self.rbm.IsHome() and ret
        #ret         = self.host.IsHome() and ret
        ret         = self.ioc.IsHome() and ret
        self.Print('Home position : %s' %str(ret))
        return ret       
    
#    def WaitFor(self):
#        "check if"
#        ret         = False
#        return ret
#    
#    def WaitForStop(self):
#        "check if"
#        ret         = False
#        return ret
#    
#    def WaitForLoadUUTToTable(self):
#        "check if"
#        ret         = False
#        return ret
#    
#    def WaitForUnloadUUTFromTable(self):
#        "check if"
#        ret         = False
#        return ret
#    
#    def WaitForLoadUUTToTestStand(self):
#        "check if"
#        ret         = False
#        return ret
#    
#    def WaitForUnloadUUTFromTestStand(self):
#        "check if"
#        ret         = False
#        return ret    
        

    ## -------------------------------
    #  -- STATES ---
    ## -------------------------------                
    def StateSpecialMessage(self, msg_in, curr_state):
        "special messages"
        msg_out     = msg_in
        next_state  = curr_state
        
        if msg_in.command == 1:
            self.Print('1 - STOP command')
            self.error = ERROR.NONE
            next_state = STATE.STOP          
        
        return msg_out, next_state
    
    def StateInit(self, msg_in, curr_state):
        "init system"
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
            self.error = ERROR.NONE
            next_state = STATE.HOME            
        
        return msg_out, next_state 
    
    def StateHome(self, msg_in, curr_state):
        "go home"
        msg_out     = msg_in
        next_state  = curr_state
            
        # do we have initial position
        #ret         = self.ActionHome()
        
        # open gripper
        self.rbm.set_gripper('open')
        
        # check is something in the griper
        #TBD  
        
            
        # check the door of the test room
        # TBD        
        
        # send table to home position
        ret         = self.ioc.SetTableHome()                
        if ret:
            self.error = ERROR.NONE
            next_state = STATE.WAIT_FOR_COMMAND
        else:
            self.error = ERROR.NO_HOME_POSITION
            next_state = STATE.ERROR 

        return msg_out, next_state     
    
    def StateWaitForCommand(self, msg_in, curr_state):
        "do onthing"
        msg_out     = msg_in
        next_state  = curr_state
        
        #if curr_state == 
        if msg_in.command == 2:
            self.Print('2 - Load UUT to index table command')
            self.error = ERROR.NONE
            next_state = STATE.LOAD_UUT_TO_TABLE
            
        elif msg_in.command == 3:
            self.Print('3 - Unload UUT from index table command')
            self.error = ERROR.NONE
            next_state = STATE.UNLOAD_UUT_FROM_TABLE    
            
        elif msg_in.command == 4:
            self.Print('4 - Load UUT to test stand command')
            self.error = ERROR.NONE
            next_state = STATE.LOAD_UUT_TO_STAND               
            
        elif msg_in.command == 5:
            self.Print('5 - Unload UUT from test stand command')          
            self.error = ERROR.NONE
            next_state = STATE.UNLOAD_UUT_FROM_STAND  
            
        elif msg_in.command == 6:
            self.Print('6 - Get staus command')            
            self.error = ERROR.NONE
            next_state = STATE.GET_AND_SEND_STATUS   
            
        elif msg_in.command == 7:
            self.Print('7 - Get bit results command')
            self.error = ERROR.NONE
            next_state = STATE.GET_AND_SEND_BIT_RESULTS               
          
        elif msg_in.command == 8:
            self.Print('8 - Connections counter zeroise command')   
            
        else:
            self.error = ERROR.BAD_HOST_COMMAND
            next_state = STATE.ERROR              
            
        return msg_out, next_state     
    
    def StateLoadUUTToTable(self, msg_in, curr_state):
        "load uut table"
        msg_out     = msg_in
        next_state  = curr_state
        
        # check if UUT in place
        
        # 2 switch rotate sensor
        
        # move table to the next index
        self.ioc.NextTableIndex()
        
        return msg_out, next_state  
    
    def StateUnLoadUUTFromTable(self, msg_in, curr_state):
        "unload uut from table"
        msg_out     = msg_in
        next_state  = curr_state
        
        # check if UUT in place - near operator
        
        # 2 switch rotate sensor
        
        # move table to the next index
        self.ioc.NextTableIndex()
        
        return msg_out, next_state   
    
    
    def StateLoadUUTToTestStand(self, msg_in, curr_state):
        "check if"
        ret         = False
        msg_out     = msg_in
        next_state  = curr_state     
        
        
        return msg_out, next_state
    
    def StateUnloadUUTFromTestStand(self, msg_in, curr_state):
        "check if"
        ret         = False
        msg_out     = msg_in
        next_state  = curr_state   
        
        # 2. open door
        ret         = self.ioc.OpenDoor()
        if not ret:
            self.Print('Tester open door timeout')
            self.error = ERROR.TESTER_OPEN_DOOR
            next_state = STATE.ERROR  
            
        # 3. check buhna
        ret        = self.ioc.BuhnaIsOpen()
        if not ret:
            self.Print('Tester open buhna timeout')
            self.error = ERROR.TESTER_OPEN_BUHNA
            next_state = STATE.ERROR  
            
        # 4. get zama out by robot
        ret        = self.rbm.GetZamaOut()
        
        # 6. take out uut
        ret        = self.rbm.GetUUTOut() 
        
        # 7. close door
        ret        = self.ioc.CloseDoor()         
        
        # 8. no UUT
        ret        = self.ioc.CheckTableNoUUT()  
            
        return msg_out, next_state
    

    def StateGetAndSendStatus(self, msg_in, curr_state):
        "check system status"
        ret         = False
        msg_out     = msg_in
        next_state  = curr_state 
        
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
        
        return msg_out, next_state    


    def StateGetAndSendBitResults(self, msg_in, curr_state):
        "check bits"
        msg_out     = msg_in
        next_state  = curr_state 
        
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
        
        return msg_out, next_state            
    
    def StateStop(self, msg_in, curr_state):
        "emergency stop"
        msg_out     = msg_in
        next_state  = curr_state
        ret         = self.Stop()
        if ret:
            self.error  = ERROR.NONE
            next_state  = STATE.FINISH
        else:
            self.error  = ERROR.CAN_NOT_STOP
            next_state  = STATE.ERROR            
        
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


    def Transition(self, msg_in):
        "transition to a different state"
        curr_state = self.state
        next_state = self.state
        
        # deal with special messages
        msg_out, curr_state = self.StateSpecialMessage(msg_in, curr_state)
        
        # deal with messages per state
        if curr_state == STATE.INIT:
            msg_out, next_state = self.StateInit(msg_in, curr_state)
            
        elif curr_state == STATE.HOME:
            msg_out, next_state = self.StateHome(msg_in, curr_state)              
            
        elif curr_state == STATE.WAIT_FOR_COMMAND:
            msg_out, next_state = self.StateWaitForCommand(msg_in, curr_state) 
            
        elif curr_state == STATE.LOAD_UUT_TO_TABLE:
            msg_out, next_state = self.StateLoadUUTToTable(msg_in, curr_state)  
            
        elif curr_state == STATE.UNLOAD_UUT_FROM_TABLE:
            msg_out, next_state = self.StateUnLoadUUTFromTable(msg_in, curr_state) 
            
        elif curr_state == STATE.LOAD_UUT_TO_STAND:
            msg_out, next_state = self.StateLoadUUTToStand(msg_in, curr_state) 
            
        elif curr_state == STATE.UNLOAD_UUT_FROM_STAND:
            msg_out, next_state = self.StateUnLoadUUTFromStand(msg_in, curr_state) 

        elif curr_state == STATE.GET_AND_SEND_STATUS:
            msg_out, next_state = self.StateGetAndSendStatus(msg_in, curr_state)             

        elif curr_state == STATE.GET_AND_SEND_BIT_RESULTS:
            msg_out, next_state = self.StateGetAndSendBitResults(msg_in, curr_state)             
                      
        elif curr_state == STATE.SPECIAL:
            msg_out, next_state = self.StateFinish(msg_in, curr_state)   
            
        elif curr_state == STATE.STOP:
            msg_out, next_state = self.StateStop(msg_in, curr_state)            
            
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
        while not self.stop:
            
            # receive message from the host
            msgRx = self.host.RecvMessage()
            
            # send message to the state machine
            msgTx  = self.Transition(msgRx)
            
            # send response to the host
            isOk   = self.host.SendMessage(msgTx)

    def Run(self):
        "running in the thread"
        self.ts = Thread(target = self.MainTask)
        self.ts.start()  

    def Print(self, txt='',level='I'):

        if level == 'I':
            ptxt = 'I: PRG: %s' % txt
            logger.info(ptxt)
        if level == 'W':
            ptxt = 'W: PRG: %s' % txt
            logger.warning(ptxt)
        if level == 'E':
            ptxt = 'E: PRG: %s' % txt
            logger.error(ptxt)
        #print(ptxt)


            
#%% Testing - unittest
class TestMainProgram:
    def __init__(self):
        from host.ComClient import ComClient as HostSimulator
        self.mp = MainProgram()
        self.hs = HostSimulator()
        
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

    def test_main(self):
        self.mp.Init()
        self.mp.Start()
        self.mp.MainTask()
        assert self.mp.state == STATE.INIT 
        
    def test_state_init(self):
        self.mp.Init()
        self.mp.Start()
        self.mp.Run()
        self.hs.TestHostSimulator(['2'])
        assert self.mp.state == STATE.INIT   
         

# --------------------------
if __name__ == '__main__':
    #from MainProgram import MainProgram
    tst = TestMainProgram()
    #tst.test_current_state() # ok
    #tst.test_init() # ok 
    #tst.test_start() # ok 
    #tst.test_main() # ???
    tst.test_state_init()