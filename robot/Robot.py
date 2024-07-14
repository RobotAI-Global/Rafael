import socket
import ast
from types import MethodType
import logging as log
import sys
#log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s',  datefmt="%M:%S")
log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")

def generate_function(function_name,address):
    def wrapped_function(self,*args,**kwargs):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(address)
        log.info('connected')
        data = {'function':function_name,'args':args,'kwargs':kwargs}
        sock.sendall(str(data).encode('utf-8'))
        log.info('sending ' + str(data))
        new_data = sock.recv(1)
        log.info('received 1') 
        new_data = sock.recv(1024)
        log.info('received ' + str(new_data))
        sock.close()
        return ast.literal_eval(new_data.decode('utf-8'))
    wrapped_function.__name__ = function_name+'_method'
    return wrapped_function


class Robot:
    def __init__(self):
        self.__server_address = ('192.168.2.13',65432)
        log.info(str(self.__server_address))
        
    def connect(self):
        self.__functions = ['move_joint','move_linear', 'move_circular', 'move_composite', 'record_path', 'power',\
            'zero_g', 'io', 'set_tool', 'gripper', 'wait', 'override', 'pause', 'unpause', 'stop', 'ik_fk', \
            'robot_status', 'program_status','get_point','get_warnings','get_errors','motion_status','initialize_attributes']
        self.__functions = ['get_point','initialize_attributes']
        for function in self.__functions:
            setattr(self, function, MethodType(generate_function(function,self.__server_address), self))
        
        for key, value in self.initialize_attributes().items():
            setattr(self,key,value)
            
    def robotInfo(self):    
        print('Robot Name: ', self.r.robot_name)
        print('Number Of Axis: ', self.r.dof)
        print('robot IP Address: ', self.r.kURL)
        print('Home Position: ', self.r.get_point("Home"))
        print('Current Joint Angles: ', self.r.robot_status("jointAngles"))
        print('Current Position: ', self.r.robot_status("cartesianPosition"))            
            
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

    def testCommands(self):
            
        self.robotPower(True)
        self.robotMode('automatic')

        # coord = []
        # coord = [0.2, -0.2, 0.2, 1.57, 0, 3]
        # coord = [200.0, -200.0, 200.0, 90.0, 0.0, 171.9745222929936]

        self.moveLinearToCoord_fromCurrentPose(-300, -300, 370, 180, 0, 90)
        self.moveLinearToPoint('CalibrationStart')       


if __name__ == '__main__':
    r = Robot()
    r.connect()
    r.testCommands()