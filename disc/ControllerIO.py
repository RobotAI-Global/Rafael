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
        
    ## ------------------------------------  
    # -- Common Interface ---
    ## ------------------------------------         
        
    def Init(self):
        ""
        self.Connect()
        self.Print('Init')
        
    def Connect(self):
        # Create a TCP/IP socket
        try:
            self.ioc1.Connect()
        except:
            self.Print('Connection to the controller 1 - fail.','W')
            return False
        
        #self.ioc1.Close()
            
        try:
            self.ioc2.Connect()
        except:
            self.Print('Connection to the controller 2 - fail.','W')  
            return False
            
        #self.ioc2.Close()
            
        self.Print('Connected to the controllers.')
        
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

    def GoHome(self):
        "defines home position for differnet devices"
        # robot is in home position already
        self.ioc1.Home()    
        self.ioc2.Home()  
        return True         
        
    def Reset(self):
        self.Print('Reset')    
        #self.ioc1.Start()    
        #self.ioc2.Start()          

    def Start(self):
        self.Print('Start')    
        self.ioc1.Start()    
        self.ioc2.Start()  
        
    def Stop(self):
        self.Print('Stop') 
        self.ioc1.Stop()    
        self.ioc2.Stop()  
        
    def Disconnect(self):
        # disonnecteing
        self.Close()   
        
    def Close(self):
        # close the connection
        self.ioc1.CloseConnectionWithController()
        self.ioc2.CloseConnectionWithController()
        
        self.Print('Connection with all controllers is closed.')        
    
 
    
    ## ------------------------------------  
    # -- Discrete Input Map HHC-1 - 105
    ## ------------------------------------ 
    def CheckEmergencyStop(self):
        "high is ok - low is emergency stop. return true when emergency stop is pressed"
        val1 = self.ioc1.GetInputStatus(1)
        ret  = val1 == 'off'
        self.Print(f'CheckEmergencyStop : {ret}')
        return ret    
    
    def CheckLockCylinder(self):
        "returns : 1 - locked, 0 - unlocked. "
        val1 = self.ioc1.GetInputStatus(2)
        ret  = val1 == 'on'
        self.Print(f'CheckLockCylinder : {ret}')
        return ret    
    
    def CheckCableInPlace(self):
        "returns : 1 - in place, 0 - not in place. "
        val1 = self.ioc1.GetInputStatus(2)
        ret  = val1 == 'on'
        self.Print(f'CheckCableInPlace : {ret}')
        return ret     
    
    def CheckTwoButtonPush(self):
        "check if 2 buttons are pushed by human opeartor - return true"
        
        ret1 = self.ioc1.GetInputStatus(4)   
        ret2 = self.ioc1.GetInputStatus(5) 

        ret    = (ret1  == 'on')  and (ret2 == 'on')
        self.Print('Pressed %s and %s' %(str(ret1), str(ret2)))
        return ret    

    def CheckLockConnectorOpen(self):
        "returns : 1 - open, 0 - not open. "
        val1 = self.ioc1.GetInputStatus(6)
        ret  = val1 == 'on'
        self.Print(f'CheckLockConnectorOpen : {ret}')
        return ret  
    
    def CheckLockConnectorClose(self):
        "returns : 1 - close, 0 -  open. "
        val1 = self.ioc1.GetInputStatus(7)
        ret  = val1 == 'on'
        self.Print(f'CheckLockConnectorClose : {ret}')
        return ret    

    def CheckAirSupply(self):
        "check if air is in - 1 - ok"
        val = self.ioc1.GetInput(8) 
        ret = val == 'on'  # 1 is ok
        self.Print(f'CheckAirSupply : {ret}')
        return ret    
    
    
    ## ------------------------------------  
    # -- Discrete Input Map HHC-2 - 106
    ## ------------------------------------     
    def CheckDoorsCylinderOpen(self):
        "return True if door open"
        val1 = self.ioc2.GetInputStatus(1)
        ret = val1 == 'on'
        self.Print(f'CheckDoorsCylinderOpen : {ret}')
        return ret
    
    def CheckDoorsCylinderClose(self):
        "return True if door close"
        val2 = self.ioc2.GetInputStatus(2)
        ret = val2 == 'on'
        self.Print(f'CheckDoorsCylinderClose : {ret}')
        return ret
    
    def CheckPartInLoadPosition(self):
        "return True if part in load position is present"
        val1 = self.ioc2.GetInputStatus(3)
        ret = val1 == 'on'
        self.Print(f'CheckPartInLoadPosition : {ret}')
        return ret   
    
    def CheckTableIndex(self):
        "return 1 if table completed the index 0-moving"
        val1    = self.ioc2.GetInputStatus(4)
        ret     = val1 == 'on'
        self.Print(f'CheckTableIndex : {ret}')
        return ret    
    
    ## ------------------------------------  
    # -- Discrete Output Map HHC-1 - 105
    ## ------------------------------------ 
    def SetDoorCylinderOpen(self):
        "set cylinder door open by 1-on,2-off"
        val1 = self.ioc1.SetOutput(1, 'on')
        val2 = self.ioc1.SetOutput(2, 'off')
        ret  = (val1 == 'on') and (val2 == 'off')
        self.Print(f'SetDoorCylinderOpen : {ret}')
        return ret   

    def SetDoorCylinderClose(self):
        "set cylinder door close by 2-on,1-off"
        val1 = self.ioc1.SetOutput(1, 'off')
        val2 = self.ioc1.SetOutput(2, 'on')
        ret  = (val1 == 'off') and (val2 == 'on')
        self.Print(f'SetDoorCylinderClose : {ret}')
        return ret     
    
    def SetLockCylinderConnectorOn(self):
        "set cylinder connector open"
        val1 = self.ioc1.SetOutput(3, 'on')
        ret  = (val1 == 'on') 
        self.Print(f'SetLockCylinderConnectorOn : {ret}')
        return ret      
    
    def SetLockCylinderConnectorOff(self):
        "set cylinder connector close "
        val1 = self.ioc1.SetOutput(3, 'off')
        ret  = (val1 == 'off') 
        self.Print(f'SetLockCylinderConnectorOff : {ret}')
        return ret    

    def SetLockCylinderOn(self):
        "set cylinder lock open"
        val1 = self.ioc1.SetOutput(4, 'on')
        ret  = (val1 == 'on') 
        self.Print(f'SetLockCylinderOn : {ret}')
        return ret      
    
    def SetLockCylinderOff(self):
        "set cylinder lock close "
        val1 = self.ioc1.SetOutput(4, 'off')
        ret  = (val1 == 'off') 
        self.Print(f'SetLockCylinderOff : {ret}')
        return ret     

  

    ## ------------------------------------  
    # -- IO Control ---
    ## ------------------------------------ 

    def Status(self):
        self.GetInputStatus()
        self.Print(f'Checking status')
        
        
    def GetInfo(self):
        # get specific bit info
        val = self.GetInputStatus(0)
        self.Print(f'IO received {val}')
        
    def SetInfo(self):
        # get specific bit info  
        addr    = 0
        val     = 1
        self.ioc.SetOutput(addr,val)
        self.Print(f'IO sending value {val} to {addr}')
        

    
    def CheckUUTInPlaceLoadPosition(self):
        "check if UUT in place for human"
        
        return True
    
    def CheckUUTInPlaceRobotPosition(self):
        "check if UUT in place for robot"
        
        return True 
    


            
    def MoveRobotLinearAxisToHomePosition(self):
        "linear axis to home position"
        
        ret     = self.MoveRobotLinearAxisBackward()
        if ret != 2:
            self.Print('Timeout  - can not reach linear axis home position','E')
            
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
                self.Print('Test Cell Door Timeout', 'E')
                break
            
        return ret  

    def OpenTestCellDoor(self):
        # opens the door of the cell
        self.Print('Open Door ...')
        
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
                self.Print('Test Cell Door Timeout', 'E')
                break
            
        return ret       
            
#    def CheckUUTInPlaceLoadPosition(self):
#        "check if UUT in place for human"
#        
#        return True
#    
#    def CheckUUTInPlaceRobotPosition(self):
#        "check if UUT in place for robot"
#        
#        return True 
#    
#    def CheckTwoButtonPush(self):
#        "check if 2 buttons are pushed by human opeartor"
#        
#        ret1 = self.GetInput(1)    
#        ret2 = self.GetInput(2) 
#        
#        self.tprint('Preassed %d and %d' %(ret1, ret2))
#        
#        ret    = ret1 and ret2
#        return ret    
#        
#    def Disconnect(self):
#        # disonnecteing
#        self.CloseConnectionWithController()  
        
        

        
    def CheckTableInHomePosition(self):
        "return true only when in IO sens the home position : Table Home Position Sensor"
        
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
            
  

    def GetInput10(self, input_id = 0):
        "reads sensor backward position of the door"  
        valOnOff    = self.GetInputStatus(input_id)
        ret         = 1 if valOnOff == 'on' else 0
        return ret     
            
            
    def GoHomeSM(self):
        "defines home position for differnet devices"
        
        # robot is in home position already
        
        # linear stage position
        ret = self.MoveRobotLinearAxisToHomePosition()
        if not ret:
            return ret
        
        # table
        ret = self.MoveTableToHomePosition()
        if not ret:
            return ret        
        
        # door
        ret = self.CloseTestCellDoor()
        if not ret:
            return ret
        
        # counter of the index
        self.uut_counter = 1
        
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
        
    def TestCheckStatusIO(self):
        #self.Connect()
        
        # turn on relay 1 with no delay
        for k in range(1):
            self.ioc1.TestScan()    
            self.ioc2.TestScan()  
            #winsound.Beep(frequency = 2500+500*k, duration = 1000)
            time.sleep(1)
            
    def TestCheckEmegencyStop(self):
        
        self.Print('Waiting for emergency press...')
        self.Connect()
        while True:
            ret = self.CheckEmergencyStop()
            if ret:
                break
        self.Print('Emergency press detected')            
        self.Close()    
          
    def TestCheckTwoButtonPush(self):
        
        self.Print('Waiting for two button press...')
        self.Connect()
        while True:
            ret = self.CheckTwoButtonPush()
            if ret:
                break
        self.Print('Button press detected')            
        self.Close()  

    def TestSetDoorCylinderOpen(self):
        self.Print('Testing door...')
        self.Connect()
        for k in range(3):
            self.SetDoorCylinderOpen()
            time.sleep(3)
            #self.SetDoorCylinderClose()
            time.sleep(3)
        #self.Print('Button press detected')            
        self.Close()                 
        
    def TestSetLockCylinderConnectorOnOff(self):
        self.Print('Testing cylinder...')
        self.Connect()
        for k in range(3):
            self.SetLockCylinderConnectorOn()
            time.sleep(3)
            self.SetLockCylinderConnectorOff()
            time.sleep(3)
        #self.Print('Button press detected')            
        self.Close()   

    def TestSetLockCylinderOnOff(self):
        self.Print('Testing cylinder...')
        self.Connect()
        for k in range(3):
            self.SetLockCylinderOn()
            time.sleep(3)
            self.SetLockCylinderOff()
            time.sleep(3)
        #self.Print('Button press detected')            
        self.Close()          
        
     
        
if __name__ == '__main__':
    c = ControllerIO()
    #c.Connect()
    #c.Test()
    #c.CheckTwoButtonPush()
    #c.CheckStatusIO()
    #c.CheckEmegencyStop()
    #c.TestCheckTwoButtonPush()
    #c.TestSetDoorCylinderOpen() # ok
    #c.TestSetLockCylinderConnectorOnOff() # ok
    c.TestSetLockCylinderOnOff()
        
        

