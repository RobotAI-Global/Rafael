from robot.Robot import Robot
import math

# Logger
import logging as log
import sys
log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")

#%% Main function
class RobotAPI:
    def __init__(self, parent=None):
        #super().__init__()
        self.parent = parent
        self.r      = Robot()
        
    def Connected(self):
        # check if the robot exists
        try:
            self.Print('Robot Name: ', self.r.robot_name)
            ret = True
        except:
            self.Print('Can not locate the robot')
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
            self.r.power(str(value))
        else:
            self.Print(f'input must be on or off - given {value}')
            
    def GetRobotStatus(self):
        sts = self.r.robot_status()
        return sts
        
    def Stop(self, value):
        # stop the robot
        self.r.stop()        
        
    def MoveJoint(self, JointAngles, j_Speed, j_Acc):
        #r.move_joint(p)
        
        # target_joint = [0.3,0.2,0.3,0,0,0]
        # r.move_joint(target_joint,speed=100,accelearation=70)
        
        # p = r.get_point("Home")
        # target_joint2 = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
        # r.move_joint(target_joint2,speed=100,accelearation=70)
        
        self.r.move_joint(JointAngles, speed = j_Speed, accelearation = j_Acc)
        self.Print("move_joint")
        
    def MoveLinear(self, Position, l_Speed, l_Acc):
        #r.move_linear(p)
        
        # target_linear = [25,30,35,0,0,0]
        # r.move_linear(target_linear,speed=0.8,accelearation=1)
        
        # p = r.get_point("Home")
        # target_Position = [p['originX'], p['originY'], p['originZ'],
        #                   p['originA'], p['originB'], p['originC']]
        # r.move_linear(target_Position,speed=0.5,accelearation=0.5)
        
        self.r.move_linear(Position, speed = l_Speed, accelearation = l_Acc)
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
        
        p = self.r.get_point(str(g_PosName))
        
        if g_Type == 'Position':            
            target = [p['originX'], p['originY'], p['originZ'],
                              p['originA'], p['originB'], p['originC']]
        elif g_Type == 'Joint':
            target = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
            
        return target  
    
    def GetCurrent(self, g_Type):
        if g_Type == 'Position':
            current = self.r.robot_status("cartesianPosition")
            
        elif g_Type == 'Joint':
            current = self.r.robot_status("jointAngles")
          
        return current
    
    def DegreeToRadians(self, deg):        

        radians = math.radians(deg)
    
        print(f"{deg} degrees = {self.radians} radians")
        
        return radians
    
    def RadiansToDegree(self, rad):
        
        degrees = math.degrees(rad)

        print(f"{rad} radians = {self.degrees} degrees")
        
        return degrees
    
    def Print(self, txt = ''):
        #print(txt)
        #log.info(txt)
        log.info(txt)    

# ------ For testing only ------
        
       
#%% Tests           
class TestRobotAPI(): #unittest.TestCase
    
    def __init__(self):
        self.r = RobotAPI()
        
                     
    def TestName(self):
        self.r.RobotData() 
        CurrentPosition = self.r.GetPosition('Position', 'Home')
        print('Current Position: ', CurrentPosition)
        CurrentJoint = self.r.GetPosition('Joint', 'Home')
        print('Current Joint: ', CurrentJoint)
        
        CurrentPosition = self.r.GetCurrent('Position')
        XPos = CurrentPosition[0]
        print('X: ', XPos)
        CurrentPosition[0] = CurrentPosition[0] + 10
        print('Current Position: ', CurrentPosition)
            
            # self.MoveLinear(CurrentPosition, 0.5, 0.5)
            
#%%
            
if __name__ == '__main__':
    from RobotAPI import TestRobotAPI as tapi
    tapi.TestName()
