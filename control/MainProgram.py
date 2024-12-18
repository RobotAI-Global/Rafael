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

import time   # just to measure switch time
#from threading import Thread

import os
import sys
parent_directory = os.path.abspath('..')
sys.path.append(parent_directory)

#base_path     = os.path.abspath(".")
#base_path      = r'D:\RobotAI\Design\apps\PickManager\gui\logo.ico' #os.path.dirname(os.path.realpath(__file__))
#iconFilePath  =  '.\\gui\\logo.ico' #os.path.join(base_path, '\logo.ico')
#base_path      = os.path.abspath(".")
#iconFilePath  =  r'C:\robotai\SW\RobotManager\gui\logo.ico' #os.path.join(base_path, '.\\gui\\logo.ico')

#import os
#import sys
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\robot')
#sys.path.insert(1, r'D:\RobotAI\Customers\VineRoboticq\Code\RobotManager\vision')



#%% 
try:
    from gui.ConfigManager import ConfigManager
    from robot.Robot import Robot as RobotManager
    from host.ComServer  import ComServer as HostManager
    from disc.ControllerIO import ControllerIO
    from gui.Logger import logger

except:
    from ..gui.ConfigManager import ConfigManager
    from ..disc.ControllerHHC import IOController
    from ..robot.Robot import Robot as RobotManager
    from ..host.ComServer  import ComServer as HostManager
    import logging
    logger      = logging.getLogger("robotai")
    print('local debug')

#%%
# import logging
# logger      = logging.getLogger("robot")

# if not logger.handlers:

# 	#log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")
# 	#log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%03(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")
# 	formatter   = logging.Formatter('[%(asctime)s] - %(levelname)s - %(message)s')
# 	logger.setLevel("DEBUG")

# 	console_handler = logging.StreamHandler()
# 	console_handler.setLevel("DEBUG")
# 	console_handler.setFormatter(formatter)
# 	logger.addHandler(console_handler)

#%% 
# StateMachine/State.py
# A State has an operation, and can be moved into the next State given an Input:
from enum import Enum
class STATE(Enum):
    INIT                    = 1
    HOME                    = 2    
    
    WAIT_FOR_COMMAND        = 20    # host command
    LOAD_UUT_TO_TABLE       = 30    # host command 
    UNLOAD_UUT_FROM_TABLE   = 40
    LOAD_UUT_TO_STAND       = 50
    UNLOAD_UUT_FROM_STAND   = 60  
    GET_AND_SEND_STATUS     = 70     # send status to host
    GET_AND_SEND_BIT_RESULTS = 71
    SPECIAL                 = 80     # deal with special message
    STOP                    = 90     # emergency stop
    FINISH                  = 100
    ERROR                   = 401
    
# An error manager:
class ERROR(Enum):
    NONE                    = 0    
    NO_CONNECTION           = 101
    NO_HOME_POSITION        = 102     # deal with special message
    CAN_NOT_STOP            = 103
    BAD_HOST_COMMAND        = 104
    UUT_NOT_IN_FRONT_ROBOT  = 105
    ROBOT_HOME_POSITION     = 106
    TESTER_OPEN_DOOR        = 201
    TESTER_OPEN_BUHNA       = 202
    MOVE_TABLE_PROBLEM      = 251
    FINISH                  = 300
    ERROR                   = 401
    EMERGENCY_STOP          = 402
    NO_AIR_SUPPLY           = 403
    
#%% message
class Message:
    def __init__(self):
        self.command    = 1
        self.data       = []
    


#%% 
class MainProgram:

    def __init__(self, parent = None):

        self.parent     = parent

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
        #self.host       = HostManager(parent = self)    
        
        # connectio to IO
        self.ioc        = ControllerIO(parent = self)
        
        logger.info('Created')
        


    ## -------------------------------
    #  -- Init All---
    ## -------------------------------
    def Init(self):
        "intialize all the modules - read some inint data from config file"
        
        #self.cfg.Init()
        self.rbm.Init()
        self.ioc.Init()
        #self.host.Init()
        #self.rsm.Init()
        self.state = STATE.INIT
        self.error = ERROR.NONE
        logger.info('Init')
        return True
        
    def Start(self):
        "start running"        
        #self.host.Start()
        self.rbm.Start()
        self.ioc.Start()
        logger.info('Start')
        return True
    
    def Home(self):
        "set system in the home"
        
        # set gripper to home position
        
        ret = self.rbm.Home()
        # if robot not in home position - abort
        if not ret:
            logger.info('Failed to put robot in home posution')
            return
        
        ret = self.ioc.Home()
        
    def Stop(self):
        "stop everything"        
        #self.host.Stop()
        self.rbm.Stop()
        self.ioc.Stop()
        logger.info('Stop')   
        return True
        
    ## -------------------------------
    #  -- Conditions ---
    ## -------------------------------   
    def CheckConnection(self):
        "check if all modules are connected"
        ret         = True
        ret         = self.rbm.IsConnected() and ret
        #ret         = self.host.IsConnected() and ret
        ret         = self.ioc.IsConnected() and ret
        logger.info('Connectivity : %s' %str(ret))
        return ret   
     
    def CheckSystemHome(self):
        "check if all modules are in home position"
        ret         = True
        ret         = self.rbm.IsHome() and ret
        #ret         = self.host.IsHome() and ret
        ret         = self.ioc.IsHome() and ret
        logger.info('Home position : %s' %str(ret))
        return ret   

    def CheckSystemAirSupply(self):
        "check if system has air"
        ret         = True
        #ret         = self.rbm.IsHome() and ret
        #ret         = self.host.IsHome() and ret
        ret         = self.ioc.CheckAirSupply() and ret
        logger.info('Air status : %s' %str(ret))
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
        

    
    ## ------------------------------------  
    # -- Table Control ---
    ## ------------------------------------        
    def MoveTableHome(self):
        # go to home position
        logger.info('Moving table to home position ...')
        
        home_sensor = self.rbm.CheckTableHomePosition()
        count       = 0
        
        # deal with table stuck in the middle
        if not home_sensor:
            ret  = self.MoveTableIndexIfBetweenStations()
        
        while not home_sensor:
             
            # move table
            self.MoveTableIndex()
            
            # debug
            time.sleep(0.2)
            
            # read home sensor again
            home_sensor = self.rbm.CheckTableHomePosition()
            
            # prevent endless rotation
            count = count + 1
            if count > 23:
                break 
            
            logger.info('Count rotations : %d' %count)
            
        if home_sensor:
            logger.info('Table in home position')
        else:
            logger.warning('Table home is not found')
            
            
        return home_sensor
    
    def MoveTableNextStation(self):
        "next station - 2 index moves"
        logger.info('Moving table to next station ...')
        
        ret = self.MoveTableIndex()
        if not ret:
            logger.warning('Can not move to the next index 1')
            return ret
        
        ret = self.MoveTableIndex()
        if not ret:
            logger.warning('Can not move to the next index 2')
            return ret
            
        logger.info('Moving table Done')
        return ret
    
    def MoveTableIndex(self, timeout = 5):
        "moving the table one index from 24 stations"
        logger.info('Moving table one index ...')  
        
        ret = self.ioc.CheckTableIsMoving()
        if ret:
            logger.warning('Table is moving or table error')
            return
        
        self.rbm.SetTableDriver('on')
        
        # catch the input goes down - table is moving
        ret = self.ioc.CheckTableIsMoving()
        t_start = time.time()
        while not ret:
            #time.sleep(0.2)
            ret = self.ioc.CheckTableIsMoving()
            if time.time() - t_start > timeout:
                logger.warning('MoveTableIndex - timeout 1')
                break        
        
        # wait for the next index
        ret = self.ioc.CheckTableReachedIndexPosition()
        t_start = time.time()
        while not ret:
            #time.sleep(0.2)
            ret = self.ioc.CheckTableReachedIndexPosition()
            if time.time() - t_start > timeout:
                logger.warning('MoveTableIndex - timeout 2')
                break
        
        # stop table rotation
        self.rbm.SetTableDriver('off')        
        
        #logger.info(f'MoveTableIndex : {ret}')
        return ret 
    
    def MoveTableIndexIfBetweenStations(self, timeout = 5):
        "moving the table when stuck between two stations - indicated by TableIsMoving"
        logger.info('Moving table one index ...')  
        
        ret1   = self.ioc.CheckTableIsMoving()
        ret2   = self.rbm.CheckTableDriverOutputOn()
        ret    = ret1 and (not ret2)
        if not ret:
            logger.info('Table is in correct position')
            return ret
        
        logger.info('Table is in the middle - fixing the position')
        
        self.rbm.SetTableDriver('on')
        
        # catch the input goes down - table is moving
        ret = self.ioc.CheckTableIsMoving()
        t_start = time.time()
        while not ret:
            #time.sleep(0.2)
            ret = self.ioc.CheckTableIsMoving()
            if time.time() - t_start > timeout:
                logger.warning('MoveTableIndex - timeout 1')
                break        
        
        # wait for the next index
        ret = self.ioc.CheckTableReachedIndexPosition()
        t_start = time.time()
        while not ret:
            #time.sleep(0.2)
            ret = self.ioc.CheckTableReachedIndexPosition()
            if time.time() - t_start > timeout:
                logger.warning('MoveTableIndex - timeout 2')
                break
        
        # stop table rotation
        self.rbm.SetTableDriver('off')        
        
        #logger.info(f'MoveTableIndex : {ret}')
        return ret     
    
    def MoveTableWithNextUUTforTest(self):
        "Find next UUT to load"
        logger.info('Moving next UUT for test ...')

        # 11. Rotate the table
        ret        = self.MoveTableNextStation()       
            
        return True

    ## ------------------------------------  
    # -- Linear Stage Control ---
    ## ------------------------------------     
    def MoveLinearCylinderBackward(self, timeout = 5):
        "moving the linear cylinder - axis 7 back"
        logger.info('Moving cylinder back ...')   
        ret = self.rbm.MoveLinearCylinderBackward(timeout = 15)
        return ret 
    
    def MoveLinearCylinderForward(self, timeout = 5):
        "moving the linear cylinder - axis 7 back"
        logger.info('Moving cylinder forward ...')  
        ret = self.rbm.MoveLinearCylinderForward(timeout = 15)
        return ret    

    ## -------------------------------
    #  -- STATES ---
    ## -------------------------------                
    def StateSpecialMessage(self, msg_in, curr_state):
        "special messages  "
        "msg_in.command == 0 - empty message"
        msg_out     = msg_in
        next_state  = curr_state
        
        if msg_in.command == 1:
            logger.info('1 - STOP command')
            self.error = ERROR.NONE
            next_state = STATE.STOP    
            
        if self.ioc.CheckEmergencyStop():
            logger.info('Emergency stop is pressed')
            self.error = ERROR.EMERGENCY_STOP
            next_state = STATE.STOP  
            
        if self.ioc.CheckAirSupply():
            logger.info('No air supply')
            self.error = ERROR.NO_AIR_SUPPLY
            next_state = STATE.STOP          
        
        return msg_out, next_state
    
    def StateInit(self, msg_in, curr_state):
        "init system"
        msg_out     = msg_in
        next_state  = curr_state
        
        self.Init()
        
#        # do we have conection
#        ret         = self.CheckConnection()
#        if not ret:
#            self.error = ERROR.NO_CONNECTION
#            next_state = STATE.ERROR
#            
#        # do we have initial position
#        ret         = self.CheckSystemState()
#        if not ret:
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
        #self.rbm.set_gripper('open')
        
        # check is something in the griper
        #TBD  
        
            
        # check the door of the test room
        # TBD        
        
        # send table to home position
        ret         = self.MoveTableHome()                
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
        if msg_in.command == 0 : # - empty message"
            pass
        
        elif msg_in.command == 1:      
            logger.info('1 - Debug')
            self.error = ERROR.NONE
            next_state = STATE.LOAD_UUT_TO_TABLE
        
        elif msg_in.command == 2:
            logger.info('2 - Load UUT to index table command')
            self.error = ERROR.NONE
            next_state = STATE.LOAD_UUT_TO_TABLE
            
        elif msg_in.command == 3:
            logger.info('3 - Unload UUT from index table command')
            self.error = ERROR.NONE
            next_state = STATE.UNLOAD_UUT_FROM_TABLE    
            
        elif msg_in.command == 4:
            logger.info('4 - Load UUT to test stand command')
            self.error = ERROR.NONE
            next_state = STATE.LOAD_UUT_TO_STAND               
            
        elif msg_in.command == 5:
            logger.info('5 - Unload UUT from test stand command')          
            self.error = ERROR.NONE
            next_state = STATE.UNLOAD_UUT_FROM_STAND  
            
        elif msg_in.command == 6:
            logger.info('6 - Get staus command')            
            self.error = ERROR.NONE
            next_state = STATE.GET_AND_SEND_STATUS   
            
        elif msg_in.command == 7:
            logger.info('7 - Get bit results command')
            self.error = ERROR.NONE
            next_state = STATE.GET_AND_SEND_BIT_RESULTS               
          
        elif msg_in.command == 8:
            logger.info('8 - Connections counter zeroise command')   
            
        else:
            self.error = ERROR.BAD_HOST_COMMAND
            next_state = STATE.ERROR              
            
        return msg_out, next_state     
    
    def StateLoadUUTToTable(self, msg_in, curr_state):
        "load uut table - done"
        msg_out     = msg_in
        next_state  = curr_state
        
        
        
        # check if UUT in place - statw wait
        ret = self.ioc.CheckPartInLoadPosition()
        while not ret:
            ret = self.ioc.CheckPartInLoadPosition()
        
        # 2 switch rotate sensor - state wait
        ret = self.ioc.CheckTwoButtonPush()
        while not ret:
            ret = self.ioc.CheckTwoButtonPush()        
        
        # move table to the next index
        ret = self.MoveTableNextStation()       
        if not ret:
            self.error = ERROR.MOVE_TABLE_PROBLEM
            next_state = STATE.ERROR 
            
        # in the final state send done to host
        
        return msg_out, next_state  
    
    def StateWaitUUTInPlace(self, msg_in, curr_state):
        "wait to load uut on table by human"
        msg_out     = msg_in
        next_state  = curr_state
        
        ret = self.ioc.CheckPartInLoadPosition()
        if ret:
            logger.info('UUT in place')
            self.error = ERROR.NONE
            next_state = STATE.WAIT_TWO_BUTTON_PRESS              
        
        return msg_out, next_state 

    def StateWaitUUTTwoButtonPress(self, msg_in, curr_state):
        "push two buttons"
        msg_out     = msg_in
        next_state  = curr_state
        
        ret = self.ioc.CheckTwoButtonPush()
        if ret:
            logger.info('Two button switch detected')
            self.error = ERROR.NONE
            next_state = STATE.NEXT_TABLE_INDE              
        
        return msg_out, next_state         
    
    def StateUnLoadUUTFromTable(self, msg_in, curr_state):
        "unload uut from table - done"
        msg_out     = msg_in
        next_state  = curr_state
        
        # check if UUT in place - state wait
        ret = self.ioc.CheckPartInLoadPosition()
        while ret: # while UUT inside
            logger.info('Wait for human to unload - PLEASE UNLOAD UUT FROM TABLE')
            ret = self.ioc.CheckPartInLoadPosition()
        
        # 2 switch rotate sensor - state wait
        ret = self.ioc.CheckTwoButtonPush()
        while not ret:
            ret = self.ioc.CheckTwoButtonPush() 
        
        # move table to the next index
        #self.ioc.NextTableIndex(increment = -1)
        ret = self.MoveTableNextStation()
        if not ret:
            self.error = ERROR.MOVE_TABLE_PROBLEM
            next_state = STATE.ERROR         
        
        return msg_out, next_state   
    
    def StateLoadUUTToStand(self, msg_in, curr_state):
        "state uut load"
        ret         = True
        msg_out     = msg_in
        next_state  = curr_state  
        
#        # check if UUT in place - state wait
#        ret = self.MoveTableWithNextUUTforTest()
#        if not ret:
#            logger.info('UUT is not in front of the robot','E')
#            self.error = ERROR.UUT_NOT_IN_FRONT_ROBOT
#            next_state = STATE.ERROR
#            return msg_out, next_state            
        
        # 0. Check UUT in front of the Robot
        ret         = self.rbm.CheckTableUnloadPosition()  
        if not ret:
            logger.warning('UUT is not in front of the robot')
            self.error = ERROR.UUT_NOT_IN_FRONT_ROBOT
            next_state = STATE.ERROR   
            return msg_out, next_state  
        
#        # 1. Check/move robot in the backward position
#        ret         = self.MoveLinearCylinderBackward()
#        if not ret:
#            logger.info('Robot linear axis is not in backward position')
#            #self.error = ERROR.ROBOT_BACKWARD_POSITION
#            next_state = STATE.ERROR 
#            
#        # 2. Check robot in the home position
#        ret         = self.rbm.CheckHomePosition() and ret
#        if not ret:
#            logger.info('Robot is not in home position')
#            self.error = ERROR.ROBOT_HOME_POSITION
#            next_state = STATE.ERROR   
#            
#        # 3. Check robot gripper in open position
#        ret         = self.rbm.CheckGripperOpen() and ret
        ret         = self.rbm.GripperClampOpen()

        
#        if not ret:
#            logger.info('Gripper is not in open position')
#            self.error = ERROR.GRIPPER_OPEN_POSITION
#            next_state = STATE.ERROR  
            
        # 6. LOAD UUT 
        ret        = self.rbm.PickUUTFromTable()      
        if not ret:
            logger.info('Robot pick from table UUT failure')
            self.error = ERROR.ROBOT_LOAD_UUT
            next_state = STATE.ERROR 
            return msg_out, next_state               
        
        # 7. open door
        ret         = self.ioc.OpenTestCellDoor() 
        if not ret:
            logger.info('Tester open door timeout')
            self.error = ERROR.TESTER_OPEN_DOOR
            next_state = STATE.ERROR 
            return msg_out, next_state              
            
#            
#        # 8. move robot forward
#        ret        = self.MoveLinearCylinderForward()  and ret         
#        if not ret:
#            logger.info('Robot move forward timeout')
#            self.error = ERROR.ROBOT_MOVE_FORWARD
#            next_state = STATE.ERROR 
            
        # 9. LOAD UUT 
        ret        = self.rbm.LoadUUTToTester()       
        if not ret:
            logger.info('Robot load UUT failure')
            self.error = ERROR.ROBOT_LOAD_UUT
            next_state = STATE.ERROR  
            return msg_out, next_state              
        
        # 11. get zama out by robot
        ret        = self.rbm.PickTestConnector() 
        if not ret:
            logger.info('Robot pick test connector')
            self.error = ERROR.ROBOT_PICK_CONNECTOR
            next_state = STATE.ERROR 
            return msg_out, next_state              
        
        # 12. plug connector
        ret        = self.rbm.PlugTestConnectorInUUT() 
        if not ret:
            logger.info('Robot plug test connector')
            self.error = ERROR.ROBOT_PLUG_CONNECTOR
            next_state = STATE.ERROR 
            return msg_out, next_state  

        # 13. gripper open
        ret         = self.rbm.GripperClampClose()      

 
        # 14. move robot home
        ret         = self.rbm.MoveRobotHomePosition() 
        if not ret:
            logger.info('Robot is not in home position')
            self.error = ERROR.ROBOT_HOME_POSITION
            next_state = STATE.ERROR   
            return msg_out, next_state              
            
#        # 10. move backward
#        ret         = self.MoveLinearCylinderBackward() and ret
#        if not ret:
#            logger.info('Robot linear axis is not in backward position')
#            self.error = ERROR.ROBOT_BACKWARD_POSITION
#            next_state = STATE.ERROR  
             
        
        # 14. close door
        ret        = self.ioc.CloseTestCellDoor() 
        if not ret:
            logger.info('Door is not closed')
            self.error = ERROR.CLOSED_DOOR
            next_state = STATE.ERROR 
        
#        # 15. Start test
#        ret       = self.host.ReadyForTest() and ret
#        if not ret:
#            logger.info('Send message')
#            self.error = ERROR.COMM_FAILURE
#            next_state = STATE.ERROR         
        
        return msg_out, next_state
    
    def StateUnloadUUTFromStand(self, msg_in, curr_state):
        "check if"
        ret         = True
        msg_out     = msg_in
        next_state  = curr_state   
        

        # 0. Check UUT in front of the Robot : 0 - UUT in the place, 1 -good
        ret         = not self.rbm.CheckTableUnloadPosition()
        if not ret:
            logger.info('Table has no place for  UUT')
            self.error = ERROR.TABLE_NO_PLACE
            next_state = STATE.ERROR  
            return msg_out, next_state              
            
        # 2. Check robot in the home position
        ret         = self.rbm.MoveRobotHomePosition() 
        if not ret:
            logger.info('Robot is not in home position')
            self.error = ERROR.ROBOT_HOME_POSITION
            next_state = STATE.ERROR          
            return msg_out, next_state              
            
        # 3. open door
        ret         = self.ioc.OpenTestCellDoor() 
        if not ret:
            logger.info('Tester open door timeout')
            self.error = ERROR.TESTER_OPEN_DOOR
            next_state = STATE.ERROR     
            return msg_out, next_state              
        
        # 4. Check robot in the forwad position
#        ret        = self.MoveLinearCylinderForward()  and ret         
#        if not ret:
#            logger.info('Robot move forward timeout')
#            self.error = ERROR.ROBOT_MOVE_FORWARD
#            next_state = STATE.ERROR 
            

        # 4. unplug connector
        ret        = self.rbm.UnPlugTestConnectorInUUT()  
        if not ret:
            logger.info('Robot unplug test connector')
            self.error = ERROR.ROBOT_PLUG_CONNECTOR
            next_state = STATE.ERROR  
            return msg_out, next_state              
            
        # 5. put connector back
        ret        = self.rbm.PutTestConnector() 
        if not ret:
            logger.info('Robot put test connector')
            self.error = ERROR.ROBOT_PICK_CONNECTOR
            next_state = STATE.ERROR 
            return msg_out, next_state    

        # 5.1 Open gripper
        ret         = self.rbm.GripperClampClose()               
            
        # 6. take out uut
        ret        = self.rbm.UnLoadUUTFromTester() 
        if not ret:
            logger.info('Robot take out UUT')
            self.error = ERROR.ROBOT_UNLOAD_UUT
            next_state = STATE.ERROR
            return msg_out, next_state              
            
        # 7. Check robot in the home position
        ret         = self.rbm.MoveRobotHomePosition()
        if not ret:
            logger.info('Robot is not in home position')
            self.error = ERROR.ROBOT_HOME_POSITION
            next_state = STATE.ERROR      
            return msg_out, next_state              
            
#        # 8. Check axis in the backward position
#        ret         = self.MoveLinearCylinderBackward() and ret
#        if not ret:
#            logger.info('Robot linear axis is not in backward position')
#            self.error = ERROR.ROBOT_BACKWARD_POSITION
#            next_state = STATE.ERROR 
            
        # 9. close door
        ret        = self.ioc.CloseTestCellDoor()      
        if not ret:
            logger.info('Close door problem')
            self.error = ERROR.CLOSE_DOOR
            next_state = STATE.ERROR 
            return msg_out, next_state              
            
        # 10. Put UUT  back
        ret        = self.rbm.PutUUTOnTable()      
        if not ret:
            logger.info('Robot put on table UUT failure')
            self.error = ERROR.ROBOT_UNLOAD_UUT
            next_state = STATE.ERROR
            return msg_out, next_state  

        # 10.5 
        ret         = self.rbm.GripperClampOpen()              
            
        # 11. Rotate the table
        ret        = self.MoveTableNextStation()      
        if not ret:
            logger.info('Rotate table failure')
            self.error = ERROR.ROBOT_UNLOAD_UUT
            next_state = STATE.ERROR            

        
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
        logger.info('Sate Finish')
        return msg_out, next_state 

    def StateError(self, msg_in, curr_state):
        "do onthing"
        msg_out         = msg_in
        next_state      = curr_state
        logger.info('ERROR STATE - DO SOMETHING')
        return msg_out, next_state  

    def Transition(self, msg_in):
        "transition to a different state"
        curr_state = self.state
        #next_state = self.state
        
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
            logger.info('Not supprted state')
            
        if curr_state != next_state:
            logger.info('Transition from %s to %s' %(str(curr_state),str(next_state)))  
            
        self.state = next_state
        return msg_out        
        
        
    ## -------------------------------
    #  -- TASKS ---
    ## -------------------------------
    def TaskStateLoadUUTToTable(self):
        "moving to stand"
        msg_in              = Message()
        curr_state          = STATE.LOAD_UUT_TO_TABLE
        msg_out, next_state = self.StateLoadUUTToTable(msg_in, curr_state) 

    def TaskStateLoadUUTToStand(self):
        "moving to stand"
        msg_in              = Message()
        curr_state          = STATE.LOAD_UUT_TO_STAND
        msg_out, next_state = self.StateLoadUUTToStand(msg_in, curr_state) 

    def TaskStateUnloadUUTFromStand(self):
        "moving to stand"
        msg_in              = Message()
        curr_state          = STATE.UNLOAD_UUT_FROM_STAND
        msg_out, next_state = self.StateUnloadUUTFromStand(msg_in, curr_state)   

    def TaskStateUnLoadUUTFromTable(self):
        "moving to stand"
        msg_in              = Message()
        curr_state          = STATE.UNLOAD_UUT_FROM_TABLE
        msg_out, next_state = self.StateUnLoadUUTFromTable(msg_in, curr_state)               

    def TaskTestStates(self, msg_in):
        "transition to a different state"
        curr_state = self.state
        #next_state = self.state
        
        # deal with special messages
        #msg_out, curr_state = self.StateSpecialMessage(msg_in, curr_state)
        
        # deal with messages per state
        if curr_state == STATE.INIT:
            msg_out, next_state = self.StateInit(msg_in, curr_state)
           
            
        elif curr_state == STATE.HOME:
            msg_out, next_state = self.StateHome(msg_in, curr_state)   
            msg_out, next_state = msg_in, STATE.LOAD_UUT_TO_TABLE
            #msg_out, next_state = msg_in, STATE.UNLOAD_UUT_FROM_TABLE
            #msg_out, next_state = msg_in, STATE.LOAD_UUT_TO_STAND
            #msg_out, next_state = msg_in, STATE.UNLOAD_UUT_FROM_STAND
            
        elif curr_state == STATE.WAIT_FOR_COMMAND:
            msg_out, next_state = self.StateWaitForCommand(msg_in, curr_state) 

       
            
        elif curr_state == STATE.LOAD_UUT_TO_TABLE:
            msg_out, next_state = self.StateLoadUUTToTable(msg_in, curr_state)  
            msg_out, next_state = msg_in, STATE.LOAD_UUT_TO_STAND
            
        elif curr_state == STATE.LOAD_UUT_TO_STAND:
            msg_out, next_state = self.StateLoadUUTToStand(msg_in, curr_state) 
            msg_out, next_state = msg_in, STATE.UNLOAD_UUT_FROM_STAND
            
        elif curr_state == STATE.UNLOAD_UUT_FROM_STAND:
            msg_out, next_state = self.StateUnloadUUTFromStand(msg_in, curr_state)
            msg_out, next_state = msg_in, STATE.UNLOAD_UUT_FROM_TABLE
            
        elif curr_state == STATE.UNLOAD_UUT_FROM_TABLE:
            msg_out, next_state = self.StateUnLoadUUTFromTable(msg_in, curr_state) 
            msg_out, next_state = msg_in, STATE.FINISH
            
#        elif curr_state == STATE.STOP:
#            msg_out, next_state = self.StateStop(msg_in, curr_state)            
#            
        elif curr_state == STATE.FINISH:
            msg_out, next_state = self.StateFinish(msg_in, curr_state)
            
        elif curr_state == STATE.ERROR:
            msg_out, next_state = self.StateError(msg_in, curr_state)     
        else:
            msg_out, next_state = msg_in, STATE.ERROR
            logger.info('Not supprted state')
            
        if curr_state != next_state:
            logger.info('Transition from %s to %s' %(str(curr_state),str(next_state)))  
            
        self.state = next_state
        
        return msg_out    
    
    
    def MainTask(self):
        "run all modules"
        
        self.state  = STATE.INIT
        msg_in      = Message()
        
        self.stop   = False
        while not self.stop:
            
            # receive message from the host
            #msgRx = self.host.RecvMessage()
            
            # send message to the state machine
            msg_out  = self.TaskTestStates(msg_in)
            
            # send response to the host
            #isOk   = self.host.SendMessage(msgTx)
            time.sleep(1)
            

    def Run(self):
        "running in the thread"
        self.ts = Thread(target = self.MainTask)
        self.ts.start()  


        
    ## -------------------------------
    #  -- TESTS ---
    ## -------------------------------    
    def TestMoveTableNextStation(self):
        "testing table motion"
        ret = self.Init()
        ret = self.MoveTableNextStation()
    
    def TestMoveTableHome(self):
        "testing table motion"
        ret = self.Init()
        ret = self.MoveTableHome()
        
        

            
#%% Testing - unittest
class TestMainHost:
    "comm with host simulator"
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
    #tst = TestMainHost()
    #tst.test_current_state() # ok
    #tst.test_init() # ok 
    #tst.test_start() # ok 
    #tst.test_main() # ???
    #tst.test_state_init()
    
    # simle programs
    mp = MainProgram()
    #mp.TestMoveTableHome()
    #mp.TestMoveTableNextStation()
    mp.MainTask()
    
    