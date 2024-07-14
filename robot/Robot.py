"""

RobotAI : Robot Neura Interface
Usage :


Usage:
    Zion -> env :  D:/RobotAI/Design/env/pose6d

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101    11.06.24 UD     Created
-----------------------------

"""


import socket
#import ast
from types import MethodType
import logging as log
import sys
import json
from threading import Thread
import time

OS = None
if sys.platform == "linux":
    import signal
    OS = "linux"
else :
    import win32api
    OS = "windows"

VERSION = "v4.12.14"
    
#import os
#LOGLEVEL = os.getenv('NEURAPY_LOG_LEVEL','WARNING')

MONITOR_CYCLE_TIME = 2


#log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")
log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%03(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")

#def generate_function(function_name,address):
#    def wrapped_function(self,*args,**kwargs):
#        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        sock.connect(address)
#        log.info('connected')
#        data = {'function':function_name,'args':args,'kwargs':kwargs}
#        sock.sendall(str(data).encode('utf-8'))
#        log.info('sending ' + str(data))
#        new_data = sock.recv(1)
#        log.info('received 1') 
#        new_data = sock.recv(1024)
#        log.info('received ' + str(new_data))
#        sock.close()
#        return ast.literal_eval(new_data.decode('utf-8'))
#    wrapped_function.__name__ = function_name+'_method'
#    return wrapped_function

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
            print(e)
            self.logger.debug("Not attaching signal handlers")
            
        self.logger.info(f"{function_name} called with args {args}, {kwargs}")
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
#    def __init__(self):
#        self.__server_address = ('192.168.2.13',65432)
#        log.info(str(self.__server_address))
    
    def __init__(self, parent=None):
        #super().__init__()
        self.parent = parent
        
        self.__server_address = ("192.168.2.13", 65432)
        self.__functions = ["get_functions","initialize_attributes"]
        self.counter = 0
        self.multiplier = 1
        self.logger = log
        for function in self.__functions:
            setattr( self, function, MethodType(generate_function(function, self.__server_address), self),)

        for key, value in self.initialize_attributes().items():
            setattr(self, key, value)
            
        for method in self.get_functions():
            if method != "list_methods":
                setattr( self,  method, MethodType(generate_function(method, self.__server_address), self), )     
        
        self.logger.info(f"Robot initialized with following functions {self.__functions} and robot version {self.version}")
        if self.version != VERSION:
            self.logger.warning(f"Current client version is not compatiable with the version of the server running on the robot. Some of the functionlities specified in the documentation might not work in the intended way. Please upgrade to the correct version .Client Version : {VERSION},Server Version : {self.version}")
        
        #self.start_diagnostics_monitor()
        
    def help(self, name):
        print(self.get_doc(name))  
    
    def print(self, txt, stam = None):
        "stam supports print line 2 arguments"
        txt = txt + str(stam) if stam is not None else txt
        self.logger.info(txt)
        
#    def connect(self):
#        self.__functions = ['move_joint','move_linear', 'move_circular', 'move_composite', 'record_path', 'power',\
#            'zero_g', 'io', 'set_tool', 'gripper', 'wait', 'override', 'pause', 'unpause', 'stop', 'ik_fk', \
#            'robot_status', 'program_status','get_point','get_warnings','get_errors','motion_status','initialize_attributes']
#        self.__functions = ['get_point','initialize_attributes']
#        for function in self.__functions:
#            setattr(self, function, MethodType(generate_function(function,self.__server_address), self))
#        
#        for key, value in self.initialize_attributes().items():
#            setattr(self,key,value)
       
        
    def list_methods(self):
        """To list the available functions in the API"""
        methods = self.get_functions()
        table = [] #PrettyTable()
        #table.field_names = ['S.No',"Function Name"]
        for index,method in enumerate(methods):
            #table.add_row([index+1,method])
            self.print(method)
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
        self.print('Robot Name: ',       self.robot_name)
        self.print('Number Of Axis: ',   self.dof)
        self.print('Robot IP Address: ', self.kURL)
        self.print('Home Position: ',    self.get_point("Home"))
        self.print('Current Joint Angles: ', self.robot_status("jointAngles"))
        self.print('Current Position: ', self.robot_status("cartesianPosition"))            
            
    def setIO(self, io_name = "DO_1", target_value = True):    # Input -> Which I/O, On/Off
        isSuccess = self.r.io("get", io_name, target_value)
        return isSuccess # Output -> I/O state
    
    def getIO(self, io_name):    # Input -> Which I/O, On/Off
        isSuccess = self.r.io("set", io_name)
        return isSuccess # Output -> I/O state
    
    def robotPower(self, pwr_switch = None):    # Input -> pwr_switch = True/False
        if pwr_switch == True:
            isSuccess = self.r.power_on()
        elif pwr_switch == False:
            isSuccess = self.r.power_off()
        else:
            print("robotMode: Wrong mode is provided: %s"%pwr_switch)

        return isSuccess
        
    def robotMode(self, rMode = "None"): # Input -> rMode = "teach"/"automatic"/"semi_automatic"
        if rMode == "teach":
            isSuccess = self.r.switch_to_teach_mode()
        elif rMode == "automatic":
            isSuccess = self.r.switch_to_automatic_mode()
        elif rMode == "semi_automatic":
            isSuccess = self.r.switch_to_semi_automatic_mode()
        else:
            print("robotMode: Wrong mode is provided: %s"%rMode)
            isSuccess = False

        return isSuccess  
    
    def moveLinearToPoint(self, pointName, speed = 0.5, acceleration = 0.2, blend_radius = 0.005):   # Input = X, Y, Z (meter); R, P, Y (radian)
        isSuccess, coordList = self.getPoint(pointName, 'Cartesian')

        if isSuccess:
            tx, ty, tz, r, p, y = coordList
            isMotionOk = self.moveLinearToCoord_fromCurrentPose(tx, ty, tz, r, p, y, speed, acceleration, blend_radius)
        else:
            isMotionOk = False
            print("moveLinearToPoint: Can not read point position")

        return isMotionOk # move is successful 
    
    def moveLinearToCoord_fromCurrentPose(self, tx, ty, tz, r, p, y, speed = 0.5, acceleration = 0.2, blend_radius = 0.005):   # Input = X, Y, Z (mm); R, P, Y (degrees)
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
    
        
    def Connected(self):
        # check if the robot exists
        try:
            self.print('Robot Name: ', self.robot_name)
            ret = True
        except:
            self.print('Can not locate the robot')
            ret = False
        return ret     
    
    def RobotData(self):    
        self.Print('Robot Name: ', self.r.robot_name)
        self.Print('Number Of Axis: ', self.r.dof)
        self.Print('robot IP Address: ', self.r.kURL)
        self.Print('Home Position: ', self.r.get_point("Home"))
        self.Print('Current Joint Angles: ', self.r.robot_status("jointAngles"))
        self.Print('Current Position: ', self.r.robot_status("cartesianPosition"))
        
    def RobotPower(self, value = 'on'):
        # r.power('on') / r.power('off')
        if value in ['on','off']:
            self.power(str(value))
        else:
            self.Print(f'input must be on or off - given {value}')
            
    def GetRobotStatus(self):
        sts = self.robot_status()
        return sts
        
    def Stop(self, value):
        # stop the robot
        self.stop()        
        
    def MoveJoint(self, JointAngles, j_Speed, j_Acc):
        #r.move_joint(p)
        
        # target_joint = [0.3,0.2,0.3,0,0,0]
        # r.move_joint(target_joint,speed=100,accelearation=70)
        
        # p = r.get_point("Home")
        # target_joint2 = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
        # r.move_joint(target_joint2,speed=100,accelearation=70)
        
        self.move_joint(JointAngles, speed = j_Speed, accelearation = j_Acc)
        self.Print("move_joint")
        
    def MoveLinear(self, Position, l_Speed, l_Acc):
        #r.move_linear(p)
        
        # target_linear = [25,30,35,0,0,0]
        # r.move_linear(target_linear,speed=0.8,accelearation=1)
        
        # p = r.get_point("Home")
        # target_Position = [p['originX'], p['originY'], p['originZ'],
        #                   p['originA'], p['originB'], p['originC']]
        # r.move_linear(target_Position,speed=0.5,accelearation=0.5)
        
        self.move_linear(Position, speed = l_Speed, accelearation = l_Acc)
        self.Print("move_linear")
        
    def GetInputStatus(self, InputName):
        #print(r.io("get",io_name="DO_1"))
        self.InputValue = self.r.io("get",io_name = str(InputName))
        
        if self.InputValue == 0:
            val = False
        else:
            val = True
            
        return val
    
    def SetOutput(self, OutputName, Action):
        # r.io("set",io_name="DO_1",target_value=True)
        
        if Action == 'on':
            self.r.io("set",io_name = str(OutputName),target_value = True)
        else:
            self.r.io("set",io_name = str(OutputName),target_value = False)
            
    def SetGripper(self, Action):
        # r.gripper("close")
        self.r.gripper(str(Action))
        
    def GetPosition(self, g_Type, g_PosName):
        #p    = r.get_point("Home")
        #p['a1'] = -p['a1']
        
        p = self.get_point(str(g_PosName))
        
        if g_Type == 'Position':            
            target = [p['originX'], p['originY'], p['originZ'],
                              p['originA'], p['originB'], p['originC']]
        elif g_Type == 'Joint':
            target = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
            
        return target  
    
    def GetCurrent(self, g_Type):
        if g_Type == 'Position':
            current = self.robot_status("cartesianPosition")
            
        elif g_Type == 'Joint':
            current = self.robot_status("jointAngles")
          
        return current
    
    def DegreeToRadians(self, deg):        

        radians = np.radians(deg)
    
        print(f"{deg} degrees = {self.radians} radians")
        
        return radians
    
    def RadiansToDegree(self, rad):
        
        degrees = np.degrees(rad)

        print(f"{rad} radians = {self.degrees} degrees")
        
        return degrees
    
    def Print(self, txt = ''):
        self.print(txt)
        #log.info(txt)
        #log.info(txt)      

    def testCommands(self):
            
        self.robotPower(True)
        self.robotMode('automatic')

        # coord = []
        # coord = [0.2, -0.2, 0.2, 1.57, 0, 3]
        # coord = [200.0, -200.0, 200.0, 90.0, 0.0, 171.9745222929936]

        self.moveLinearToCoord_fromCurrentPose(-300, -300, 370, 180, 0, 90)
        self.moveLinearToPoint('CalibrationStart') 
        
    def test_move_linear(self): #, Position, l_Speed, l_Acc):
        #r.move_linear(p)
        
        # target_linear = [25,30,35,0,0,0]
        # r.move_linear(target_linear,speed=0.8,accelearation=1)
        
         p = self.get_point("Home")
         target_Position = [p[:6]]
         self.move_linear(target_Position,speed=0.5,accelearation=0.5)         

#%% Tests           
class TestRobotAPI: #unittest.TestCase
    
    def __init__(self):
        self.r = Robot()
 
    def TestMotion(self):
        self.r.robot_info() 
        #r.list_methods()
        self.r.power_on()
        self.r.switch_to_automatic_mode()
        self.r.robot_info()
        
        p = self.r.get_current_cartesian_pose()
        self.r.print(p)
        p[0] = p[0] - 0.1
    
        self.r.move_linear_from_current_position([p], 5, 1)      
                     
    def TestName(self):
        self.r.robot_info() 
        CurrentPosition = self.r.GetPosition('Position', 'Home')
        print('Current Position: ', CurrentPosition)
        CurrentJoint = self.r.GetPosition('Joint', 'Home')
        print('Current Joint: ', CurrentJoint)
        
        CurrentPosition = self.r.GetCurrent('Position')
        XPos = CurrentPosition[0]
        print('X: ', XPos)
        CurrentPosition[0] = CurrentPosition[0] + 10
        print('Current Position: ', CurrentPosition)

#%%
if __name__ == '__main__':
    
    #from Robot import TestRobotAPI
    tapi = TestRobotAPI()
    #tapi.TestName()    
    tapi.TestMotion()
    
    
#    r = Robot()
#    #r.list_methods()
#    r.power_on()
#    r.switch_to_automatic_mode()
#    r.robot_info()
#    
#    p = r.get_current_cartesian_pose()
#    r.print(p)
#    p[0] = p[0] - 0.1
#
#    r.move_linear_from_current_position([p], 5, 1)
#    #r.move_linear(p, 5, 1)
#    #r.test_move_linear()