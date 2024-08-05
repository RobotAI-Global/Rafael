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
from robot.RobotAPI import RobotAPI as RobotManager
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
        

    ## -------------------------------
    #  -- Init All---
    ## -------------------------------
    def initModules(self):
        "intialize all the modules - read some inint data from config file"
        self.cfg.init()
        self.rbm.Init()
        self.ioc.Init()
        self.host.Init()
        self.rsm.init()
        
    def startModules(self):
        "start running"
        
        self.host.RunThread()
        self.rbm.RunThread()
        self.ioc.RunThread()
        
        
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


    def MainTask(self):    
        global stopTask         
        
        while not stopTask:
            
            # Check if recived queue is empty
            if not self.s.queueToRobot.empty():
                
                # received packet from comm server            
                pcktToRobot    = self.s.queueToRobot.get()
                
                # sending packet to robot thread
                self.r.queueToRobot.put(pcktToRobot)  
                print('Sending Data from Server Thread to Robot Thread')
            
            # Check if sending queue is empty
            if not self.r.queueFromRobot.empty():
    
                # extract results
                pcktFromRobot = self.r.queueFromRobot.get()
                
                # send to server
                self.s.queueFromRobot.put(pcktFromRobot)
                print('Sending Data from Robot Thread to Server Thread')       
    
        print('Exit Main Task')
        
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
    from MainProgram import MainProgram
    m = MainProgram()
    m.mainTask()
