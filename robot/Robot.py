"""

RobotAI : Robot Neura Interface

Login:
     basicuser

Usage :


Usage:
    Zion -> env :  D:/RobotAI/Design/env/pose6d

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0301    03.11.24 UD     Define In/Out
0202    12.09.24 UD     IO interface
0201    23.08.24 UD     New robot interface
0101    11.06.24 UD     Created
-----------------------------

"""


import socket
#import ast
from types import MethodType
#import logging as log
import sys
import os
#from logging.handlers import TimedRotatingFileHandler
import json
from threading import Thread
import time
import numpy as np
#from Logger import logger

OS = None
if sys.platform == "linux":
    import signal
    OS = "linux"
else :
    import win32api
    OS = "windows"

VERSION = "v4.17.9"
    
LOGLEVEL = os.getenv('NEURAPY_LOG_LEVEL','WARNING')

MONITOR_CYCLE_TIME = 2

SOCKET_ADDRESS = os.getenv("SOCKET_ADDRESS", "192.168.2.13")
SOCKET_PORT = 65432

#%% Logger

import logging
logger      = logging.getLogger("robot")

#%%

##log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")
##log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%03(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")
#formatter   = logging.Formatter('[%(asctime)s] - [%(filename)12s:%(lineno)3d] - %(levelname)s - %(message)s')
#logger.setLevel("DEBUG")
#
#console_handler = logging.StreamHandler()
#console_handler.setLevel("DEBUG")
#console_handler.setFormatter(formatter)
#logger.addHandler(console_handler)

#class CustomFormatter(logging.Formatter):
#
#    grey = "\x1b[38;20m"
#    yellow = "\x1b[33;20m"
#    red = "\x1b[31;20m"
#    bold_red = "\x1b[31;1m"
#    reset = "\x1b[0m"
#    format = (
#        "[%(asctime)s][%(name)s][%(levelname)s] : %(message)s :(%(filename)s:%(lineno)d)"
#    )
#
#    FORMATS = {
#        logging.DEBUG: grey + format + reset,
#        logging.INFO: grey + format + reset,
#        logging.WARNING: yellow + format + reset,
#        logging.ERROR: red + format + reset,
#        logging.CRITICAL: bold_red + format + reset,
#    }
#
#    def format(self, record):
#        log_fmt = self.FORMATS.get(record.levelno)
#        formatter = logging.Formatter(log_fmt,datefmt="%Y-%m-%d %H:%M:%S")
#        return formatter.format(record)
#
#
#def get_console_handler():
#    console_handler = logging.StreamHandler(sys.stdout)
#    console_handler.setLevel(eval('logging.'+LOGLEVEL))
#    console_handler.setFormatter(CustomFormatter())
#    return console_handler
#
#
#def get_logger(logger_name):
#    logger = logging.getLogger(logger_name)
#    #if not logger.hasHandlers():
#    logger.addHandler(get_console_handler())
#    print('Console')
#    logger.setLevel(logging.DEBUG)
#    logger.propagate = False
#    return logger
#
#neurapy_logger = get_logger("neurapy_logger")


#%% Main


def generate_function(function_name, address):
    def wrapped_function(self, *args, **kwargs):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(address)
        except ConnectionError:
            raise ConnectionError("Failed to establish the communication to control box.Please cross check whether the robot is reachable or try reset control from Teach pendant")
        try:
            if OS == 'linux':
                signal.signal(signal.SIGINT,lambda signal, frame: self.stop())
                signal.signal(signal.SIGHUP,lambda signal, frame: self.stop())
            else:
                win32api.SetConsoleCtrlHandler(self.stop, True)
        except Exception as e:
            self.logger.debug("Not attaching signal handlers")
            print(e)
            
        #self.logger.info(f"{function_name} called with args {args}, {kwargs}")
        data = {"function": function_name, "args": args, "kwargs": kwargs}
        sock.sendall(json.dumps(data).encode("utf-8"))
        new_data = sock.recv(8192)
        sock.close()
        response = json.loads(new_data.decode("utf-8"))
        if response["error"]:
            self.logger.error(f"{function_name} call with args {args}, {kwargs}, failed with exception {response['error']}")
            raise Exception(response["error"])
        return response["result"]

    wrapped_function.__name__ = function_name + "_method"
    return wrapped_function


class Robot:
    def __init__(self, parent=None):
        #super().__init__()
        self.parent             = parent

        self.__server_address   = (SOCKET_ADDRESS, SOCKET_PORT)
        self.__functions        = ["get_functions","initialize_attributes"]
        
        self.counter            = 0
        self.multiplier         = 1
        self.logger             = logger
        
        self.is_alive           = False
        
        self.tprint('Robot IF is created')
        
    def connect(self):
        "start communication"
        
        #self.ROBOT_HOME_POSE    = []
        
        for function in self.__functions:
            setattr(
                self,
                function,
                MethodType(generate_function(function, self.__server_address), self),
            )

        for key, value in self.initialize_attributes().items():
            setattr(self, key, value)
        for method in self.get_functions():
            if method != "list_methods":
                setattr(
                    self,
                    method,
                    MethodType(generate_function(method, self.__server_address), self),
                )     
        self.logger.info(f"Robot initialized with following functions {self.__functions} and robot version {self.version}")
        if self.version != VERSION:
            self.logger.warning(f"Current client version is not compatiable with the version of the server running on the robot. Some of the functionlities specified in the documentation might not work in the intended way. Please upgrade to the correct version .Client Version : {VERSION},Server Version : {self.version}")
        
        #self.start_diagnostics_monitor()
        
        self.tprint('Collision is disabled')
        self.disable_collision_detection()
        
        self.tprint('Automatic mode')
        self.power_on()
        self.switch_to_automatic_mode()
        
        self.tprint('Connetcion Created')
        
    def help(self, name):
        print(self.get_doc(name))
    
    def tprint(self, txt, stam = None):
        "stam supports print line 2 arguments"
        txt = txt + str(stam) if stam is not None else txt
        self.logger.info(txt)
        #print(txt)
        
        
    def list_methods(self):
        """To list the available functions in the API"""
        methods = self.get_functions()
        table = [] #PrettyTable()
        #table.field_names = ['S.No',"Function Name"]
        for index, method in enumerate(methods):
            #table.add_row([index+1,method])
            self.tprint(method)
        return table
        
    def notify_diagnostics(self):
        while True:
            diagnostics = self.get_diagnostics()
            if "critical" in diagnostics:
                if diagnostics["critical"]:
                    self.counter += 1
                    if self.counter > 1000*self.multiplier:
                        self.logger.error(f"{diagnostics}")
                        self.counter = 0
                        self.multiplier += 1
            time.sleep(MONITOR_CYCLE_TIME)
        
    def start_diagnostics_monitor(self):
        self.monitor = Thread(target=self.notify_diagnostics)
        self.monitor.daemon = True
        self.monitor.start()
        

    def robot_info(self):     
        self.tprint('Robot Name: ',       self.robot_name)
        self.tprint('Number Of Axis: ',   self.dof)
        self.tprint('Robot IP Address: ', self.kURL)
        self.tprint('Home Position: ',    self.get_point("Home"))
        self.tprint('Current Joint Angles: ', self.robot_status("jointAngles"))
        self.tprint('Current Position: ', self.robot_status("cartesianPosition"))            
            
    def set_io(self, io_name = "DO_1", target_value = True):    # Input -> Which I/O, On/Off
        isSuccess = self.r.io("get", io_name, target_value)
        return isSuccess # Output -> I/O state
    
    def get_io(self, io_name):    # Input -> Which I/O, On/Off
        isSuccess = self.r.io("set", io_name)
        return isSuccess # Output -> I/O state
    
    def robot_power(self, pwr_switch = None):    # Input -> pwr_switch = True/False
        if pwr_switch == True:
            isSuccess = self.r.power_on()
        elif pwr_switch == False:
            isSuccess = self.r.power_off()
        else:
            self.tprint("robotMode: Wrong mode is provided: %s"%pwr_switch)

        return isSuccess
        
    def robot_mode(self, rMode = "None"): # Input -> rMode = "teach"/"automatic"/"semi_automatic"
        if rMode == "teach":
            isSuccess = self.r.switch_to_teach_mode()
        elif rMode == "automatic":
            isSuccess = self.r.switch_to_automatic_mode()
        elif rMode == "semi_automatic":
            isSuccess = self.r.switch_to_semi_automatic_mode()
        else:
            self.tprint("robotMode: Wrong mode is provided: %s"%rMode)
            isSuccess = False

        return isSuccess  
    
    def move_linear_to_point(self, pointName, speed = 0.5, acceleration = 0.2, blend_radius = 0.005):   # Input = X, Y, Z (meter); R, P, Y (radian)
        isSuccess, coordList = self.r.get_point(pointName, 'Cartesian')

        if isSuccess:
            tx, ty, tz, r, p, y = coordList
            isMotionOk = self.move_linear_to_coord_from_current_pose(tx, ty, tz, r, p, y, speed, acceleration, blend_radius)
        else:
            isMotionOk = False
            print("moveLinearToPoint: Can not read point position")

        return isMotionOk # move is successful 
    
    def move_linear_to_coord_from_current_pose(self, tx, ty, tz, r, p, y, speed = 0.5, acceleration = 0.2, blend_radius = 0.005):   # Input = X, Y, Z (mm); R, P, Y (degrees)
        coordList = []
        coordList.extend((tx, ty, tz, r, p, y))
        coordList = self.mmd_To_mr(coordList)
        
        #   Debug print
        print("Debug: moveLinearToCoord_fromCurrentPose: Coordinate that given to move: %s"%(str(coordList)))

        linear_property = {
                "speed": speed,
                "acceleration": acceleration,
                "blend_radius": blend_radius,
                "target_pose": [coordList]
                }
        
        isMotionOk = self.r.move_linear_from_current_position(**linear_property)

        return isMotionOk  # move is successful 
    
    # ---------------------------------
    # additional interfaces    
    def is_connected(self):
        # check if the robot exists
        try:
            self.tprint('Robot Name: ', self.robot_name)
            ret = True
        except:
            self.tprint('Can not locate the robot')
            ret = False
        return ret     
    
#    def RobotData(self):   
#        
#        self.tprint('Robot Name: ', self.r.robot_name)
#        self.tprint('Number Of Axis: ', self.r.dof)
#        self.tprint('robot IP Address: ', self.r.kURL)
#        self.tprint('Home Position: ', self.r.get_point("Home"))
#        self.tprint('Current Joint Angles: ', self.r.robot_status("jointAngles"))
#        self.tprint('Current Position: ', self.r.robot_status("cartesianPosition"))
        
#    def RobotPower(self, value = 'on'):
#        # r.power('on') / r.power('off')
#        if value in ['on','off']:
#            self.power(str(value))
#        else:
#            self.tprint(f'input must be on or off - given {value}')
            
    def get_robot_status(self):
        sts = self.robot_status()
        return sts
     
        
#    def MoveJoint(self, JointAngles, j_Speed, j_Acc):
#        #r.move_joint(p)
#        
#        # target_joint = [0.3,0.2,0.3,0,0,0]
#        # r.move_joint(target_joint,speed=100,accelearation=70)
#        
#        # p = r.get_point("Home")
#        # target_joint2 = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
#        # r.move_joint(target_joint2,speed=100,accelearation=70)
#        
#        self.move_joint(JointAngles, speed = j_Speed, accelearation = j_Acc)
#        self.tprint("move_joint")
        
#    def MoveLinear(self, Position, l_Speed, l_Acc):
#        #r.move_linear(p)
#        
#        # target_linear = [25,30,35,0,0,0]
#        # r.move_linear(target_linear,speed=0.8,accelearation=1)
#        
#        # p = r.get_point("Home")
#        # target_Position = [p['originX'], p['originY'], p['originZ'],
#        #                   p['originA'], p['originB'], p['originC']]
#        # r.move_linear(target_Position,speed=0.5,accelearation=0.5)
#        
#        self.r.move_linear_from_current_position(Position, speed = l_Speed, accelearation = l_Acc)
#        self.tprint("move_linear")
        
    def get_digital_io_input(self, InputName = 1):
        #print(r.io("get",io_name="DO_1"))
        #self.InputValue = self.io("get",io_name = str(InputName))
        val = self.get_digital_input(InputName)
        
#        if self.InputValue == 0:
#            val = False
#        else:
#            val = True
#        self.tprint(val)     
        return val
    
    def get_digital_io_output(self, InputName = 1):
        #print(r.io("get",io_name="DO_1"))
        #self.InputValue = self.io("get",io_name = str(InputName))
        val = self.get_digital_output(InputName)
        
#        if self.InputValue == 0:
#            val = False
#        else:
#            val = True
#        self.tprint(val)    
        return val    
    
    def set_digital_io_output(self, OutputName, Action):
        # Output is not connected
        if OutputName is None:
            return False
        
        if not isinstance(OutputName, int):
            return False  
        
        if OutputName < 0 or OutputName > 7:
            return False        
        
        # r.io("set",io_name="DO_1",target_value=True)
        if isinstance(Action,str):
            Action = True if Action == 'on' else False
            
        io_set = self.set_digital_output(OutputName,Action)
        #self.tprint(io_set)
        return io_set

            
    def get_analog_io(self, input_id = 1):
        "analog io"
        io_get = self.get_analog_input(input_id)
        self.tprint(io_get)
            
    def set_gripper(self, Action):
        # r.gripper("close")
        self.gripper(str(Action))
        
    def get_gripper(self):
        "gripper status"
        ret = self.gripper()
        return ret
        
    def get_point_pose(self, g_Type = "Position", pointName = "Home"):
        "g_Type - Position or Joint"

        
        # list of joint angles
        #target       = self.get_point(str(g_PosName))
        target       = self.get_point(pointName, 'Cartesian')
        
#        if g_Type == 'Position':            
#            target = [p['originX'], p['originY'], p['originZ'], p['originA'], p['originB'], p['originC']]
#        elif g_Type == 'Joint':
#            target = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
            
        #self.tprint(str(target))
        return target  
    
    def get_current_pose(self, g_Type = 'Position'):
        if g_Type == 'Position':
            current = self.robot_status("cartesianPosition")
            
        elif g_Type == 'Joint':
            current = self.robot_status("jointAngles")
          
        quart   = current[3:7]
        rpy     = self.quaternion_to_rpy(quart[0],quart[1],quart[2],quart[3])
        print('RPY' , rpy)
        return current
    
    def get_pose_euler(self):
        "get pose in mm and euler rotation angles"
        p       = self.get_current_cartesian_pose()        
        quart   = p[3:7]
        rpy     = self.quaternion_to_rpy(quart[0],quart[1],quart[2],quart[3])
        rpy_deg = [a*180/np.pi for a in rpy]
        p_mm    = [pp*1000 for pp in p[:3]]
        pose    = [p_mm[0], p_mm[1], p_mm[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]]
        self.tprint('Euler pose: ' , pose)
        return pose    
    
    def set_pose_euler(self, pose):
        "set pose in mm and euler rotation angles"
        self.tprint('Euler pose: ' , pose)     
        rpy     = [a/180*np.pi for a in pose[3:6]]
        quat     = self.rpy_to_quaternion(rpy[0],rpy[1],rpy[2])
        p_m      = [pp/1000 for pp in pose[:3]]
        p        = [p_m[0], p_m[1], p_m[2], quat[0], quat[1], quat[2], quat[3]]
        res      = self.move_linear_from_current_position([p], 5, 1)
        return True  
    
    
    ## -------------------------------
    #  -- INTERFACES ---
    ## -------------------------------     
    def Init(self):
        "compatability"
        self.connect()

    def Start(self):
        "compatability"
        self.power_on()
        self.switch_to_automatic_mode()  
        self.Print('Start')
        
    def Stop(self, value):
        # stop the robot
        self.stop()   
        
    def IsConnected(self):
        # chck if the robot is there
        ret = self.is_connected()         
        return ret
    
    def IsHome(self):
        "check default position"
        pose = self.get_pose_euler()
        return True
    
    # ----------------------------
    # robot discrete inputs 
    # ----------------------------    
    def CheckTableHomePosition(self):
        "check input THP"
        val = self.get_digital_io_input(0)
        ret = val > 0.5
        #self.tprint(f'CheckTableHomePosition : {ret}')
        return ret
    
    def CheckTableUnloadPosition(self):
        "check input PIPL"
        val = self.get_digital_io_input(1)
        ret = val > 0.5
        self.tprint(f'CheckTableUnloadPosition : {ret}')
        return ret  
    
    def CheckGripperPush(self):
        "check input GRHP"
        val1 = self.get_digital_io_input(3)
        ret  = val1 > 0.5 
        self.tprint(f'CheckGripperPush : {ret}')
        return ret  
    
    def CheckGripperPull(self):
        "check input GRHP"
        val1 = self.get_digital_io_input(2)
        ret  = val1 > 0.5 
        self.tprint(f'CheckGripperPull : {ret}')
        return ret     
    
    def CheckLinearCylinderForward(self):
        "check linear cylinder forward"
        val = self.get_digital_io_input(5)
        ret  = val > 0.5
        self.tprint(f'CheckLinearCylinderForward : {ret}')
        return ret 
    
    def CheckLinearCylinderBackward(self):
        "check linear cylinder"
        val1 = self.get_digital_io_input(4)
        ret  = val1 > 0.5
        self.tprint(f'CheckLinearCylinderBackward : {ret}')
        return ret     
    
    def CheckGripperClampClose(self):
        "check input GRCL"
        val1 = self.get_digital_io_input(7)
        ret  = val1 > 0.5
        self.tprint(f'CheckGripperClampClose : {ret}')
        return ret  
    
    def CheckGripperClampOpen(self):
        "check input GRCL"
        val1 = self.get_digital_io_input(6)
        ret  = val1 > 0.5 
        self.tprint(f'CheckGripperClampOpen : {ret}')
        return ret  
    
    def CheckTableDriverOutputOn(self):
        "check if digital output is active"
        val1 = self.get_digital_io_output(3)
        ret  = val1 > 0.5 
        self.tprint('CheckTableDriverOutputOn : %s' %str(ret))
        return ret     
    
    # ----------------------------
    # robot discrete outputs 
    # ----------------------------  
    def SetLinearCylinderForward(self):
        "set linear cylinder forward"
        val1 = self.set_digital_io_output(0, 'on') # none
        val2 = self.set_digital_io_output(1, 'off')
        ret  = val1 and val2 
        self.tprint(f'SetLinearCylinderForward : {ret}')
        return ret  
    
    def SetLinearCylinderBackward(self):
        "set linear cylinder backward"
        val1 = self.set_digital_io_output(0, 'off')
        val2 = self.set_digital_io_output(1, 'on')
        ret  = val1 and val2 
        self.tprint(f'SetLinearCylinderBackward : {ret}')
        return ret 

    def SetTableDriver(self, on_off = 'off'):
        "enable one index table"
        val1 = self.set_digital_io_output(3, on_off)
        ret  = val1 
        #self.tprint(f'SetTableDriver : {ret}')
        return ret  
    
    def SetGripperClampOpen(self):
        "open clamp/gripper"
        val1 = self.set_digital_io_output(4, 'off')
        val2 = self.set_digital_io_output(5, 'on')
        ret  = val1 and val2 
        self.tprint(f'SetGripperClampOpen : {ret}')
        return ret    
    
    def SetGripperClampClose(self):
        "close clamp/gripper"
        val1 = self.set_digital_io_output(4, 'on')
        val2 = self.set_digital_io_output(5, 'off')
        ret  = val1 and val2 
        self.tprint(f'SetGripperClampClose : {ret}')
        return ret 

    def SetGripperCoverPush(self):
        "open cover "
        val1 = self.set_digital_io_output(6, 'on')
        ret  = val1  
        self.tprint(f'SetGripperCoverPush : {ret}')
        return ret    
    
    def SetGripperCoverPull(self):
        "open cover pull"
        val1 = self.set_digital_io_output(6, 'off')
        ret  = val1  
        self.tprint(f'SetGripperCoverPull : {ret}')
        return ret  
    
    def SetGripperMembraneOn(self):
        "open cover "
        val1 = self.set_digital_io_output(7, 'on')
        ret  = val1  
        self.tprint(f'SetGripperMembraneOn : {ret}')
        return ret    
    
    def SetGripperMembraneOff(self):
        "open cover pull"
        val1 = self.set_digital_io_output(7, 'off')
        ret  = val1  
        self.tprint(f'SetGripperMembraneOff : {ret}')
        return ret     

    # ----------------------------
    # robot linear stage functionality 
    # ----------------------------  
    def MoveLinearCylinderForward(self, timeout = 5):
        "set linear cylinder forward and wait for reaching the position"
        ret = self.SetLinearCylinderForward() # 
        ret = self.CheckLinearCylinderForward()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckLinearCylinderForward()
            if time.time() - t_start > timeout:
                self.tprint('MoveLinearCylinderForward - timeout')
                break
        
        self.tprint(f'MoveLinearCylinderForward : {ret}')
        return ret 

    def MoveLinearCylinderBackward(self, timeout = 5):
        "set linear cylinder backward and wait for reaching the position"
        # timeout = self.TIMEOUT_CYLINDER
        ret = self.SetLinearCylinderBackward() # 
        ret = self.CheckLinearCylinderBackward()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckLinearCylinderBackward()
            if time.time() - t_start > timeout:
                self.tprint('MoveLinearCylinderForward - timeout')
                break
        
        self.tprint(f'MoveLinearCylinderForward : {ret}')
        return ret 
         
    # ----------------------------
    # gripper functionality 
    # ----------------------------  
    def GripperClampOpen(self, timeout = 5):
        "set clamp open forward and wait for reaching the position"
        ret = self.SetGripperClampOpen() # 
        ret = self.CheckGripperClampOpen()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckGripperClampOpen()
            if time.time() - t_start > timeout:
                self.tprint('GripperClampOpen - timeout')
                break
        
        self.tprint(f'GripperClampOpen : {ret}')
        return ret     
    
    def GripperClampClose(self, timeout = 5):
        "set clamp close forward and wait for reaching the position"
        ret = self.SetGripperClampClose() # 
        ret = self.CheckGripperClampClose()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckGripperClampClose()
            if time.time() - t_start > timeout:
                self.tprint('GripperClampClose - timeout')
                break
        
        self.tprint(f'GripperClampClose : {ret}')
        return ret 

    def GripperCoverPush(self, timeout = 5):
        "set gripper cover push"
        ret = self.SetGripperCoverPush() # 
        ret = self.CheckGripperPush()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckGripperPush()
            if time.time() - t_start > timeout:
                self.tprint('GripperCoverPush - timeout')
                break
        
        self.tprint(f'GripperCoverPush : {ret}')
        return ret 

    def GripperCoverPull(self, timeout = 5):
        "set gripper cover pull"
        ret = self.SetGripperCoverPull() # 
        ret = self.CheckGripperPull()
        t_start = time.time()
        while not ret:
            time.sleep(0.2)
            ret = self.CheckGripperPull()
            if time.time() - t_start > timeout:
                self.tprint('GripperCoverPull - timeout')
                break
        
        self.tprint(f'GripperCoverPull : {ret}')
        return ret      

    # ----------------------------
    # robot motion control
    # ----------------------------
    
    def GetHomePosition(self):
        "get default position using home point"
        pose = self.get_point_pose( 'Position', 'Home')
        return pose  
    
    def GetCurrentPosition(self):
        "get current position "
        pose = self.get_current_pose()
        return pose       

    def MoveRobotHomePosition(self):
        "check the home position"
        pose_home = self.GetHomePosition()
        pose_curr = self.GetCurrentPosition()
        
        ret       = np.all(pose_home == pose_curr)
        return True
    
    def CheckAirCution(self):
        "air cution"
        #self.set_gripper('open')
        ret = True 
        return ret 
    
    def CheckPushCylinder(self):
        "push cylinder"
        #self.set_gripper('open')
        ret = True 
        return ret     
    
    def CheckGripperOpen(self):
        "is gripper open"
        ret = self.CheckAirCution()
        ret = self.CheckPushCylinder() and ret
        
        self.set_gripper('open')        
        return ret
    
    def MovePathPoints(self, point_list = []):
        "executes motion over defined point list"
        if len(point_list) < 1:
            point_list = ['Home', 'AboveTable','TakePart','AboveTable','InfrontDoor','InfrontTestStand','TestStand']
            
        point_list_reverse = point_list[::-1]
        repeat_num = 1
        for k in range(repeat_num):
            for pname in point_list:
                next_pose = self.get_point_pose('Position', pname)
                self.move_linear_from_current_position([next_pose], 5, 1)
                self.tprint('Finished : %s' %pname)
                time.sleep(0.1)
                
#            for pname in point_list_reverse:
#                next_pose = self.get_point_pose('Position', pname)
#                self.move_linear_from_current_position([next_pose], 5, 1) 
#                self.tprint('Finished : %s' %pname)
#                time.sleep(0.1)                
#                
        self.tprint('MovePathPoints done')
        

    def PickTestConnector(self):
        "picking test connector"
        ret = True
        self.Print('Pick Test Connector')  
        return ret
    
    def PutTestConnector(self):
        "put test connector"
        ret = True
        self.Print('Put Test Connector')  
        return ret        
    
    def PlugTestConnectorInUUT(self):
        "plug test connector"
        ret = True
        self.Print('Plug Test Connector')  
        
        # move
        
        # release gripper
        
        
        return ret 
    
    def UnPlugTestConnectorInUUT(self):
        "unplug"
        ret = True
        self.Print('UnPlug Test Connector')  
        
        # move
        
        # close gripper
                
        return ret     
    

    def GetUUTOut(self):
        "compatability"
        ret = True
        self.Print('GetUUTOut')  
        return ret 
    
    def PutUUTOnTable(self):
        "put UUT back to table"
        point_list = ['Home', 'AboveTable','TakePart','AboveTable','Home']
        self.MovePathPoints(point_list)
        
        # open griper
        
        # close gripper
        
        return True     
        
    def PickUUTFromTable(self):
        "pick UUT to be tested"
        #pose = self.get_point_pose(g_Type = "Position", g_PosName = "Home")
        #self.set_pose(pose)
        
        point_list = ['Home', 'AboveTable','TakePart','AboveTable','InfrontDoor']
        self.MovePathPoints(point_list)


        # open griper
        
        # close gripper
        
        return True         
    
    def LoadUUTToTester(self):
        "loads UUT to tester"
        #pose = self.get_point_pose(g_Type = "Position", g_PosName = "Home")
        #self.set_pose(pose)
        
        point_list = ['InfrontDoor','InfrontTestStand','TestStand']
        self.MovePathPoints(point_list)
        
        # lock UUT on tester
        
        return True  

    def Print(self, txt='',level='I'):
        ptxt = 'ROB: %s' % str(txt)
        if level == 'I':
            logger.info(ptxt)
        if level == 'W':
            logger.warning(ptxt)
        if level == 'E':
            logger.error(ptxt)
        
        #print(ptxt)        

#%% Tests           
class TestRobotAPI: #unittest.TestCase
    
    def __init__(self):
        self.r = Robot()
        self.r.connect()
        
    def TestInfo(self):
        
        self.r.robot_info() 
        self.r.list_methods()
        d = self.r.get_diagnostics()
        print(d)
        
        # need one argument
        #prop = self.r.get_doc()
        #print(prop)        
        
        #prop = self.r.get_current_tool_properties()
        #print(prop)
        
        prop = self.r.get_tools()
        print(prop)        
        
        currentPosition = self.r.get_point_pose('Position', 'Home')
        print('Current Position: ', currentPosition)
        
        currentJoint = self.r.get_point_pose('Joint', 'Home')
        print('Current Joint: ', currentJoint)
        
        currentPosition = self.r.get_current_pose('Position')
        print('Current Position: ', currentPosition)
        XPos            = currentPosition[0]
        print('X: ', XPos)
        #currentPosition[0] = currentPosition[0] + 10
        #print('Current Position: ', currentPosition)     
        #self.r.stop()
 
    def TestMotion(self):
        self.r.robot_info() 
        self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        #self.r.robot_info()
        
        p = self.r.get_current_cartesian_pose()
        self.r.tprint(p)
        p[0] = p[0] - 0.1
    
        self.r.move_linear_from_current_position([p], 5, 1)      
        
    def TestCommands(self):
        self.r.robot_info() 
        self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        
        
        p = self.r.get_current_cartesian_pose()
        self.r.tprint(p)
        p[0] = p[0] - 0.1
    
        self.r.move_linear_from_current_position([p], 5, 1)          
                     
    def TestEulerMotion(self):
        self.r.robot_info() 
        #self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        
        pose = self.r.get_pose_euler()
        pose[0] = pose[0] + 10 # mm
        pose[5] = pose[5] + 10 # deg
        isOK = self.r.set_pose_euler(pose)
        
    def TestSwitchTool(self):
        self.r.robot_info() 
        #self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        
        pose = self.r.get_pose_euler()
 
        prop = self.r.get_tools()
        param= prop[1]
        self.r.set_tool(param)
        
        pose = self.r.get_pose_euler()
       
    def TestGripper(self):
        #self.r.robot_info() 
        #self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        
        gr   = self.r.get_gripper()
        print(gr)
        
        self.r.set_gripper('open')
        time.sleep(1)
        self.r.set_gripper('close')
        time.sleep(1)
        self.r.set_gripper('open')
        
    def TestIO(self):
        #self.r.robot_info() 
        #self.r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        
        print('Digital')
        gr   = self.r.get_digital_io_input()
        print(gr)
        gr   = self.r.get_digital_io_input()
        print(gr)
        
        self.r.set_digital_io_output(1,'on')
        time.sleep(1)
        self.r.set_digital_io_output(1,'off')
        
        print('Analog')
        gr   = self.r.get_analog_io()
        print(gr)
        
    def TestPathMotion(self):
        "move between differnet points defined in the robot"
        
        #self.r.power_on()
        #self.r.switch_to_automatic_mode()
        
#        point_list = ['Home', 'AboveTable','TakePart','AboveTable']
#        repeat_num = 5
#        for k in range(repeat_num):
#            for pname in point_list:
#                next_pose = self.r.get_point_pose('Position', pname)
#                self.r.move_linear_from_current_position([next_pose], 5, 1) 
#                time.sleep(1)
        
        self.r.MovePathPoints()
                
        print('TestPathMotion')
        

    def TestFunctionalInputs(self):
        "actual pin names"
        for k in range(3):
            ret = self.r.CheckTableHomePosition()
            ret = self.r.CheckTableUnloadPosition()
            ret = self.r.CheckGripperPush()
            ret = self.r.CheckLinearCylinder()
            ret = self.r.CheckGripperClamp()

            time.sleep(1)
            
            
    def TestSetLinearCylinderForwardBackward(self):
        "linear cylinder"
        for k in range(3):
            self.r.SetLinearCylinderForward()
            time.sleep(3)
            self.r.SetLinearCylinderBackward()
            time.sleep(3)
            
    def TestMoveLinearCylinderForwardBackward(self):
        "linear cylinder motion"
        for k in range(3):
            self.r.MoveLinearCylinderForward()
            time.sleep(3)
            self.r.MoveLinearCylinderBackward()
            time.sleep(3)

    def TestSetTableDriver(self):
        "test index table"
        for k in range(3):
            self.r.SetTableDriver('on')
            time.sleep(1)
            self.r.CheckTableDriverOutputOn()
            time.sleep(3)
            self.r.SetTableDriver('off')
            time.sleep(1)
            self.r.CheckTableDriverOutputOn()
            time.sleep(3)
            
    def TestGripperFunctions(self):
        "test gripper ios"
        for k in range(3):    
            
            self.r.SetGripperClampOpen()
            self.r.SetGripperClampClose()
            self.r.SetGripperCoverPush()
            self.r.SetGripperCoverPull()
            self.r.SetGripperMembraneOn()
            self.r.SetGripperMembraneOff()
       
  

#%%
if __name__ == '__main__':
    
    #from Robot import TestRobotAPI
    tapi = TestRobotAPI()
    #tapi.TestInfo()    # ok
    #tapi.TestMotion() # ok
    #tapi.TestCommands() # ok
    #tapi.TestEulerMotion() # ok
    #tapi.TestSwitchTool() # 
    #tapi.TestGripper() # 
    #tapi.TestIO() # 
    #tapi.TestPathMotion()
    #tapi.TestFunctionalInputs() # ok
    #tapi.TestSetLinearCylinderForwardBackward() # ok
    #tapi.TestMoveLinearCylinderForwardBackward() # ok
    tapi.TestSetTableDriver()
    #tapi.TestGripperFunctions()
