"""

RobotAI : IO Controller - uses two HHC cards to control all the IO

Login:
     

Usage :


Usage:
    Zion -> env :  D:/RobotAI/Design/env/pose6d

-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0302    09.11.24 UD     logger improve
0301    11.10.24 UD     2 x IO interface. New IP
0202    12.09.24 UD     IO interface
0101    11.08.24 UD     Created
-----------------------------

"""


#import socket
#from queue import Queue
#from threading import Thread
import time
try:
    from disc.ControllerHHC import IOController
    from gui.Logger import logger
    print('main app')
except:
    from ControllerHHC import IOController
    import logging
    logger      = logging.getLogger("robotai")
    print('local debug')



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
    # -- API Interface ---
    ## ------------------------------------         
        
    def Init(self):
        ""
        self.Connect()
        logger.info('Init')
        
    def Connect(self):
        # Create a TCP/IP socket
        try:
            self.ioc1.Connect()
        except:
            logger.info('Connection to the controller 1 - fail.','W')
            return False
        
        #self.ioc1.Close()
            
        try:
            self.ioc2.Connect()
        except:
            logger.info('Connection to the controller 2 - fail.','W')  
            return False
            
        #self.ioc2.Close()
            
        logger.info('Connected to the controllers.')
        
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
        ret1 = self.ioc1.Default()    
        ret2 = self.ioc2.Default()  
        return ret1 and ret2         
        
    def Reset(self):

        logger.info('Reset')    
        self.GoHome()    
        #self.ioc2.Start()          

    def Start(self):
        logger.info('Start')    
        self.ioc1.Start()    
        self.ioc2.Start()  
        
    def Stop(self):
        logger.info('Stop') 
        self.ioc1.Stop()    
        self.ioc2.Stop()  
        
    def Disconnect(self):
        # disonnecteing
        self.Close()   
        
    def Close(self):
        # close the connection
        self.ioc1.Close()
        self.ioc2.Close()
        
        logger.info('Connection with all controllers is closed.')        
    
    def Status(self):
        self.GetInputStatus()
        logger.info(f'Checking status')
        
    def GetInfo(self):
        # get specific bit info
        val = self.GetInputStatus(0)
        logger.info(f'IO received {val}')
        
            
    ## ------------------------------------  
    # -- Discrete Input Map HHC-1 - 105
    ## ------------------------------------
     
    def CheckEmergencyStop(self):
        "high is ok - low is emergency stop. return true when emergency stop is pressed"
        val1 = self.ioc1.GetInputStatus(1)
        ret  = val1 == 'off'
        logger.info(f'CheckEmergencyStop : {ret}')
        return ret    
    
    def CheckLockCylinder(self):
        "returns : 1 - locked, 0 - unlocked. "
        val1 = self.ioc1.GetInputStatus(2)
        ret  = val1 == 'on'
        logger.info(f'CheckLockCylinder : {ret}')
        return ret    
    
    def CheckCableInPlace(self):
        "returns : 1 - in place, 0 - not in place. "
        val1 = self.ioc1.GetInputStatus(2)
        ret  = val1 == 'on'
        logger.info(f'CheckCableInPlace : {ret}')
        return ret     
    
    def CheckTwoButtonPush(self):
        "check if 2 buttons are pushed by human opeartor - return true"
        
        ret1 = self.ioc1.GetInputStatus(4)   
        ret2 = self.ioc1.GetInputStatus(5) 

        ret    = (ret1  == 'on')  and (ret2 == 'on')
        #logger.info('Pressed %s and %s' %(str(ret1), str(ret2)))
        return ret    

    def CheckLockConnectorOpen(self):
        "returns : 1 - open, 0 - not open. "
        val1 = self.ioc1.GetInputStatus(6)
        ret  = val1 == 'on'
        logger.info(f'CheckLockConnectorOpen : {ret}')
        return ret  
    
    def CheckLockConnectorClose(self):
        "returns : 1 - close, 0 -  open. "
        val1 = self.ioc1.GetInputStatus(7)
        ret  = val1 == 'on'
        logger.info(f'CheckLockConnectorClose : {ret}')
        return ret    

    def CheckAirSupply(self):
        "check if air is in - 1 - ok"
        val = self.ioc1.GetInput(8) 
        ret = val == 'on'  # 1 is ok
        logger.info(f'CheckAirSupply : {ret}')
        return ret    
    
    
    ## ------------------------------------  
    # -- Discrete Input Map HHC-2 - 106
    ## ------------------------------------ 
    #     
    def CheckDoorsCylinderOpen(self):
        "return True if door open"
        val1 = self.ioc2.GetInputStatus(1)
        ret = val1 == 'on'
        logger.info(f'CheckDoorsCylinderOpen : {ret}')
        return ret
    
    def CheckDoorsCylinderClose(self):
        "return True if door close"
        val2 = self.ioc2.GetInputStatus(2)
        ret = val2 == 'on'
        logger.info(f'CheckDoorsCylinderClose : {ret}')
        return ret
    
    def CheckPartInLoadPosition(self):
        "return True if part in load position is present"
        val1 = self.ioc2.GetInputStatus(3)
        ret = val1 == 'on'
        #logger.info(f'CheckPartInLoadPosition : {ret}')
        return ret   
    
    def CheckTableReachedIndexPosition(self):
        "return 1 if table completed the index 0-moving"
        val1    = self.ioc2.GetInputStatus(4)
        ret     = val1 == 'off'
        #logger.info(f'CheckTableReachedIndexPosition : {ret}')
        return ret   
    
    def CheckTableIsMoving(self):
        "return 1 if table is moving"
        val1    = self.ioc2.GetInputStatus(4)
        ret     = val1 == 'on'
        #logger.info(f'CheckTableIsMoving : {ret}')
        return ret       
    
    ## ------------------------------------  
    # -- Discrete Output Map HHC-1 - 105
    ## ------------------------------------ 
    def SetDoorCylinderOpen(self):
        "set cylinder door open by 1-on,2-off"
        val1 = self.ioc1.SetOutput(1, 'on')
        val2 = self.ioc1.SetOutput(2, 'off')
        ret  = (val1 == 'on') and (val2 == 'off')
        logger.info(f'SetDoorCylinderOpen : {ret}')
        return ret   

    def SetDoorCylinderClose(self):
        "set cylinder door close by 2-on,1-off"
        val1 = self.ioc1.SetOutput(1, 'off')
        val2 = self.ioc1.SetOutput(2, 'on')
        ret  = (val1 == 'off') and (val2 == 'on')
        logger.info(f'SetDoorCylinderClose : {ret}')
        return ret     
    
    def SetLockCylinderConnectorOn(self):
        "set cylinder connector open"
        val1 = self.ioc1.SetOutput(3, 'on')
        ret  = (val1 == 'on') 
        logger.info(f'SetLockCylinderConnectorOn : {ret}')
        return ret      
    
    def SetLockCylinderConnectorOff(self):
        "set cylinder connector close "
        val1 = self.ioc1.SetOutput(3, 'off')
        ret  = (val1 == 'off') 
        logger.info(f'SetLockCylinderConnectorOff : {ret}')
        return ret    

    def SetLockCylinderOn(self):
        "set cylinder lock open"
        val1 = self.ioc1.SetOutput(4, 'on')
        ret  = (val1 == 'on') 
        logger.info(f'SetLockCylinderOn : {ret}')
        return ret      
    
    def SetLockCylinderOff(self):
        "set cylinder lock close "
        val1 = self.ioc1.SetOutput(4, 'off')
        ret  = (val1 == 'off') 
        logger.info(f'SetLockCylinderOff : {ret}')
        return ret     


    ## ------------------------------------  
    # -- Functional Control ---
    ## ------------------------------------ 

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
            logger.info('Timeout  - can not reach linear axis home position','E')
            
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
                logger.info('Test Cell Door Timeout')
                break
            
        return ret  

    def OpenTestCellDoor(self):
        # opens the door of the cell
        logger.info('Open Door ...')
        
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
                logger.info('Test Cell Door Timeout', 'E')
                break
            
        return ret       
            
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
                logger.info('Move Backward Timeout', 'E')
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
                logger.info('Move Backward Timeout', 'E')
                break
            
        return ret          
            
    def MoveRobotLinearAxisToHomePosition(self):
        "linear axis to home position"
        
        ret     = self.MoveRobotLinearAxisBackward()
        if ret != 2:
            logger.info('Timeout  - can not reach linear axis home position','E')
            
    ## ---------------
    # Tests
    ## ---------------

    def Test(self):
        #self.Connect()
        # turn on relay 1 with no delay
        self.ioc1.Test()    
        self.ioc2.Test()  
        
        self.Close()   
        
    def TestCheckStatusIO(self):
        #self.Connect()
        
        # turn on relay 1 with no delay
        for k in range(3):
            self.ioc1.TestScan()    
            self.ioc2.TestScan()  
            #winsound.Beep(frequency = 2500+500*k, duration = 1000)
            time.sleep(1)
            
    def TestCheckEmegencyStop(self):
        
        logger.info('Waiting for emergency press...')
        self.Connect()
        while True:
            ret = self.CheckEmergencyStop()
            if ret:
                break
        logger.info('Emergency press detected')            
        self.Close()    
          
    def TestCheckTwoButtonPush(self):
        
        logger.info('Waiting for two button press...')
        self.Connect()
        while True:
            ret = self.CheckTwoButtonPush()
            if ret:
                break
        logger.info('Button press detected')            
        self.Close()  

    def TestSetDoorCylinderOpen(self):
        logger.info('Testing door...')
        self.Connect()
        for k in range(3):
            self.SetDoorCylinderOpen()
            time.sleep(3)
            #self.SetDoorCylinderClose()
            time.sleep(3)
        #logger.info('Button press detected')            
        self.Close()                 
        
    def TestSetLockCylinderConnectorOnOff(self):
        logger.info('Testing cylinder...')
        self.Connect()
        for k in range(3):
            self.SetLockCylinderConnectorOn()
            time.sleep(3)
            self.SetLockCylinderConnectorOff()
            time.sleep(3)
        #logger.info('Button press detected')            
        self.Close()   

    def TestSetLockCylinderOnOff(self):
        logger.info('Testing cylinder...')
        self.Connect()
        for k in range(3):
            self.SetLockCylinderOn()
            time.sleep(3)
            self.SetLockCylinderOff()
            time.sleep(3)
        #logger.info('Button press detected')            
        self.Close()          
        
     
        
if __name__ == '__main__':
    c = ControllerIO()
    #c.Connect()
    #c.Test()
    #c.CheckTwoButtonPush()
    c.TestCheckStatusIO()
    #c.CheckEmegencyStop()
    #c.TestCheckTwoButtonPush()
    #c.TestSetDoorCylinderOpen() # ok
    #c.TestSetLockCylinderConnectorOnOff() # ok
    #c.TestSetLockCylinderOnOff()
        
        

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
#        logger.info('Preassed %d and %d' %(ret1, ret2))
#        
#        ret    = ret1 and ret2
#        return ret    
#        
#    def Disconnect(self):
#        # disonnecteing
#        self.CloseConnectionWithController()  
        
       