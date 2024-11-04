"""

RobotAI : IO Controller - uses two HHC cards to control all the IO

Login:
     

Usage :


Usage:
    Zion -> env :  D:/RobotAI/Design/env/pose6d

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0301    11.10.24 UD     2 x IO interface. New IP
0202    12.09.24 UD     IO interface
0101    11.08.24 UD     Created
-----------------------------

"""

#import socket
#from queue import Queue
#from threading import Thread
import time
from disc.ControllerHHC import IOController
#import winsound

#%% Logger
import logging
logger      = logging.getLogger("robot")

#%%
class ControllerIO:
    def __init__(self, parent=None):

        self.parent         = parent

        # connectio to IO
        self.ioc1           = IOController(self, '192.168.2.105', 5000)
        
        # connectio to IO
        self.ioc2           = IOController(self, '192.168.2.106', 6000)        
        
        # time out variable
        self.TIMEOUT_CYLINDER = 10  # sec        
        
    def Init(self):
        ""
        self.Home()
        self.tprint('Init')
        
    def Connect(self):
        # Create a TCP/IP socket
        try:
            self.ioc1.Connect()
        except:
            self.tprint('Connection to the controller 1 - fail.','W')
            return False
        
        self.ioc1.Close()
            
        try:
            self.ioc2.Connect()
        except:
            self.tprint('Connection to the controller 2 - fail.','W')  
            return False
            
        self.ioc2.Close()
            
        self.tprint('Connected to the controllers.')
        
        return True
    
    def IsConnected(self):
        # chekc if the socket is open

        ret1 = self.ioc1.IsConnected() 
        ret2 = self.ioc2.IsConnected() 
        return ret1 and ret2  
     
    def IsHome(self):
        "check if the controller in the home position"
        ret1 = self.ioc1.IsHome()    
        ret2 = self.ioc2.IsHome()          
        return ret1 and ret2          
        

    def Reset(self):
        self.tprint('Reset')    
        #self.ioc1.Start()    
        #self.ioc2.Start()          

    def Start(self):
        self.tprint('Start')    
        self.ioc1.Start()    
        self.ioc2.Start()  
        
    def Stop(self):
        self.tprint('Stop') 
        self.ioc1.Stop()    
        self.ioc2.Stop()  
        
    def Disconnect(self):
        # disonnecteing
        self.Close()   
        
    def Close(self):
        # close the connection
        self.ioc1.CloseConnectionWithController()
        self.ioc2.CloseConnectionWithController()
        
        self.tprint('Connection with all controllers is closed.')        
    
    def Home(self):
        "defines home position for differnet devices"
        
        # robot is in home position already
        self.ioc1.Home()    
        self.ioc2.Home()  
        
        return True
    
    ## ------------------------------------  
    # -- Discrete Input Map ---
    ## ------------------------------------ 
    def CheckDoorsCylinderOpen(self):
        "check input HHC-1"
        val1 = self.ioc1.GetInputStatus(1)
        val2 = self.ioc1.GetInputStatus(1)
        ret = val1 > 0.5 and val2 > 0.5
        self.tprint(f'CheckTableHomePosition : {ret}')
        return ret

    
    ## ------------------------------------  
    # -- Table Control ---
    ## ------------------------------------        
    def SetTableHome(self):
        # go to home position
        self.tprint('Starting Homing ...')
        
        home_sensor = False
        while not home_sensor:
             
            # read home sensor
            home_sensor = True
        
        return True
        
    def GetTableIndex(self):
        # go to home position
        self.tprint('Current table position ...')
        return True
    
    def NextTableIndex(self, increment = 1):
        # go to next position
        self.tprint('Next table position ...')
        
        # two stations
        self.MoveTableIndex()
        self.MoveTableIndex()

        return True
    
    ## ------------------------------------  
    # -- Tester Control ---
    ## ------------------------------------        

    def BuhnaIsOpen(self):
        # buhna is open
        self.tprint('Open Buhna ...')
        return True     

    ## ------------------------------------  
    # -- IO Control ---
    ## ------------------------------------ 

    def Status(self):
        self.GetInputStatus()
        self.tprint(f'Checking status')
        
        
    def GetInfo(self):
        # get specific bit info
        val = self.GetInputStatus(0)
        self.tprint(f'IO received {val}')
        
    def SetInfo(self):
        # get specific bit info  
        addr    = 0
        val     = 1
        self.ioc.SetOutput(addr,val)
        self.tprint(f'IO sending value {val} to {addr}')
        
    def CheckAirSupply(self):
        "check if air is in"
         
        val = self.ioc2.GetInput(3) 
        ret = val > 0.5  # 1 is ok
        return ret
    
    def CheckEmergencyStop(self):
        "check if emergency is pressed"
        
        # set output to move backward
        #self.SetOutput(1, '00')   
        return True 
    
    def CheckUUTInPlaceLoadPosition(self):
        "check if UUT in place for human"
        
        return True
    
    def CheckUUTInPlaceRobotPosition(self):
        "check if UUT in place for robot"
        
        return True 
    
    def CheckTwoButtonPush(self):
        "check if 2 buttons are pushed by human opeartor"
        
        ret1 = self.ioc1.GetInput(4)   
        ret2 = self.ioc1.GetInput(5) 
        
        self.tprint('Pressed %s and %s' %(str(ret1), str(ret2)))
        
        ret    = ret1  > 0.5 and ret2 > 0.5
        return ret    

        
    def LinearAxisForwardPosition(self):
        "reads sensor forward position of the linear state"
        
        return True
        
    def LinearAxisBackwardPosition(self):
        "reads sensor backward position of the linear state"  
        
        return True
        
    def GetRobotLinearAxisPosition(self):
        """ 
        
        linear stage
        
        Returns:
            ret : 0 - not defined, 1 - forward position, 2 - backward position
        
        """
        
        ret = 0
        if self.LinearAxisForwardPosition():
            ret = 1
        elif self.LinearAxisBackwardPosition():
            ret = 2
        
        return ret
        
    def CheckTableInHomePosition(self):
        "return true only when in IO sens the home position : Table Home Position Sensor"
        
        return True
    
    
    def SetTableIndex(self):
        "moves the table for one index"
        
        return True
    
    def WaitForTableIndexDone(self):
        "the table has reached index poxition and finished to move"
        
        return True
    
    def MoveTableIndex(self):
        "rotates table by one position"
        " the table has actual 12 statoins but totally is build for 24 indexes"
        self.SetTableIndex()
        ret = False
        while not ret:            
            ret = self.WaitForTableIndexDone()        
        return True    
        
    def MoveTableToHomePosition(self):
        "rotate table to home position"
        ret = self.CheckTableInHomePosition()
        while not ret:
            self.MoveTableIndex()
            ret = self.CheckTableInHomePosition()
            
    def MoveRobotLinearAxisBackward(self):
        "moves robot to backward position"
        
        # set output to move backward
        #self.SetOutput(1, '00')
        
        ret = 0
        t_start = time.time()
        timeout = self.TIMEOUT_CYLINDER
        while not ret == 2:  
            # stage is moving
            time.sleep(1)  # 
            
            # check back poition gain
            ret     = self.GetRobotLinearAxisPosition()
            
            if (time.time() - t_start) > timeout:
                self.tprint('Move Backward Timeout', 'E')
                break
            
        return ret    

    def MoveRobotLinearAxisForward(self):
        "moves robot to forward position"
        
        # set output to move backward
        #self.SetOutput(1, '00')
        
        ret = 0
        t_start = time.time()
        timeout = self.TIMEOUT_CYLINDER
        while not ret == 1:  
            # stage is moving
            time.sleep(1)  # 
            
            # check back poition gain
            ret     = self.GetRobotLinearAxisPosition()
            
            if (time.time() - t_start) > timeout:
                self.tprint('Move Backward Timeout', 'E')
                break
            
        return ret          
            
    def MoveRobotLinearAxisToHomePosition(self):
        "linear axis to home position"
        
        ret     = self.MoveRobotLinearAxisBackward()
        if ret != 2:
            self.tprint('Timeout  - can not reach linear axis home position','E')
            
    def GetTestCellDoorForwardPosition(self):
        "reads sensor forward position of the door"
        
        return True
        
    def GetTestCellDoorBackwardPosition(self):
        "reads sensor backward position of the door"  
        
        return True            
            
    def GetTestCellDoorSensor(self):
        """ 
        door position piston
        Returns:
            ret : 0 - not defined, 1 - closed, 2 - open
        
        """
        
        ret = 0
        if self.GetTestCellDoorForwardPosition():
            ret = 1
        elif self.GetTestCellDoorBackwardPosition():
            ret = 2
        
        return ret        
            
    def CloseTestCellDoor(self):
        "closes the door of the cell"
        
        # set output to move forward
        #self.SetOutput(1, '00')
        
        ret = 0
        t_start = time.time()
        timeout = self.TIMEOUT_CYLINDER
        while not ret == 2:  
            # stage is moving
            time.sleep(1)  # 
            
            # check back poition gain
            ret     = self.GetTestCellDoorSensor()
            
            if (time.time() - t_start) > timeout:
                self.tprint('Test Cell Door Timeout', 'E')
                break
            
        return ret  

    def OpenTestCellDoor(self):
        # opens the door of the cell
        self.tprint('Open Door ...')
        
        # set output to move backward
        #self.SetOutput(1, '00')
        
        ret = 0
        t_start = time.time()
        timeout = self.TIMEOUT_CYLINDER
        while not ret == 1:  
            # stage is moving
            time.sleep(1)  # 
            
            # check back poition gain
            ret     = self.GetTestCellDoorSensor()
            
            if (time.time() - t_start) > timeout:
                self.tprint('Test Cell Door Timeout', 'E')
                break
            
        return ret       
            

             

    def Print(self, ptxt='',level='I'):
        
        if level == 'I':
            #ptxt = 'I: IOC: %s' % txt
            logger.info(ptxt)
        if level == 'W':
            #ptxt = 'W: IOC: %s' % txt
            logger.warning(ptxt)
        if level == 'E':
            #ptxt = 'E: IOC: %s' % txt
            logger.error(ptxt)
            
        print(ptxt)
            
        
    def Test(self):
        #self.Connect()
        
        # turn on relay 1 with no delay
        self.ioc1.Test()    
        self.ioc2.Test()  
        
        self.Close()   
        
    def CheckStatusIO(self):
        #self.Connect()
        
        # turn on relay 1 with no delay
        for k in range(1):
            self.ioc1.TestScan()    
            self.ioc2.TestScan()  
            #winsound.Beep(frequency = 2500+500*k, duration = 1000)
            time.sleep(1)
        
        #self.Close()         
        
if __name__ == '__main__':
    c = ControllerIO()
    #c.Connect()
    #c.Test()
    #c.CheckTwoButtonPush()
    c.CheckStatusIO()
        
        

