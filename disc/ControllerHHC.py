"""

RobotAI : IO Interface

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

import socket
#from queue import Queue
from threading import Thread
import time

#%% Logger
import logging
logger      = logging.getLogger("robot")

#%%
class IOController:
    def __init__(self, parent=None, host = '192.168.2.106', port = 5000):
        #super().__init__()
        self.parent         = parent
        # Server information
        self.ServerIp       = host  # IP address of the controller
        self.ServerPort     = port  # port number of the controller  
        self.client_socket  = None
        self.ts             = None   # thread
        self.stopTask       = False
        
        # time out variable
        self.TIMEOUT_CYLINDER = 10  # sec
        
        # count of stations : 1 upto 12 - total 12, 0 - undefined
        self.uut_counter   = 0
        
    def Init(self):
        
        self.uut_counter   = 0
        self.Print('Init')
        
        
    def Connect(self):
        # Create a TCP/IP socket
        self.client_socket  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # connect to the server, if result = 0 connection is OK
        result              = self.client_socket.connect((self.ServerIp, self.ServerPort))
        self.Print('Connected to the controller.')
        
        return result
    
    def IsConnected(self):
        # chekc if the socket is open
        ret = False
        if self.client_socket is None:
            return ret
        return True
    
    def IsHome(self):
        # chekc if the IOs are in correct position
        ret = True
        return ret    
        
    def SendDataToController(self, cCommand):
        # Send data to the server       
        self.client_socket.sendall(cCommand.encode())
        self.Print('Message sent to the controller:', cCommand)        
    
    def ReciveDataFromController(self, cCommand, cNumber):
        # Receive data from the server
        self.data = self.client_socket.recv(1024).decode()
        self.Print('Message received from the controller:', self.data)       

        if cCommand == 'input':
            # Take only the data from the return string 'input00000000'
            # The order of the inputs is: 76543210
            self.InputStatus = self.data.replace('input', '') 
            
            # Reverse the order of a string to get the order of inputs: 01234567
            self.ReversedInputStatus = self.InputStatus[::-1]
            
            # Get the input status
            self.iStatus = self.ReversedInputStatus[cNumber-1] 
            
            if self.iStatus == '1':
                self.bStatus = 'on'                
            elif self.iStatus == '0':
                self.bStatus = 'off'                            

        elif cCommand == 'output':
            # remove all numbers from a string and leve only alpha
            self.bStatus = ''.join(char for char in self.data if char.isalpha())                                                
            
        return str(self.bStatus)
    
    def GetInputStatus(self, iNumber):
        # read all Inputs
        self.SendDataToController('input')
        # get input status
        self.rValue = self.ReciveDataFromController('input', iNumber)
        self.Print('GetInputStatus : %s' %str(iNumber))
        return self.rValue
    
    def SetOutput(self, iNumber, sDelay):
        self.sCommand = 'on'
        self.OutputNumber = str(iNumber)
        
        # Sample Commands
        # turn on relay 1
        # self.SendDataToController('on1')
        # turn on relay 1 with delay of 5 sec
        # self.SendDataToController('on1:05')        
        # turn on relay for 5sec (up to 99sec)
        
        if sDelay == '00' :            
            self.sOutput = self.sCommand + self.OutputNumber
        else:
            self.sOutput = self.sCommand + self.OutputNumber + ':' + str(sDelay)
            
        # turn on relay 
        self.SendDataToController(self.sOutput) 
        # get relay status
        self.rValue = self.ReciveDataFromController('output', iNumber)
        
        self.Print('SetOutput : %s' %str(iNumber))
        return self.rValue
    
    def ResetOutput(self, iNumber):
        self.sCommand = 'off'
        self.OutputNumber = str(iNumber)
        
        # Sample Commands
        # turn off relay 1
        # self.SendDataToController('off1')
                         
        self.sOutput = self.sCommand + self.OutputNumber
                    
        # turn on relay 
        self.SendDataToController(self.sOutput) 
        # get relay status
        self.rValue = self.ReciveDataFromController('output', iNumber)
        
        self.Print('ResetOutput : %s' %str(iNumber))
        return self.rValue
        
    def CloseConnectionWithController(self):
        # close the connection
        self.client_socket.close()
        self.Print('Connection with the controller closed.')
        
    def ControllerThread(self):  
        "main thread"        
        while not self.stopTask:
            try:
                self.Connect()    
                print('Server is running')
                while not self.stopTask:
                    
                    self.ReciveData()
                    time.sleep(0.5)
                    self.SendData()
                    
            except Exception as e:
                print(e)  
                
    def RunThread(self):
        "running in the thread"
        self.ts = Thread(target = self.ControllerThread)
        self.ts.start()
        #self.ts.join()    
        return True 
    
    def Start(self):
        self.Print('Start')    
        self.RunThread()
        
    def Stop(self):
        self.Print('Stop') 
        self.stopTask = True
        self.ts.join()          
    
    ## ------------------------------------  
    # -- Table Control ---
    ## ------------------------------------        
    def SetTableHome(self):
        # go to home position
        self.Print('Starting Homing ...')
        
        home_sensor = False
        while not home_sensor:
             
            # read home sensor
            home_sensor = True
        
        return True
        
    def GetTableIndex(self):
        # go to home position
        self.Print('Current table position ...')
        return True
    
    def NextTableIndex(self, increment = 1):
        # go to next position
        self.Print('Next table position ...')
        
        # two stations
        self.MoveTableIndex()
        self.MoveTableIndex()
  
        self.uut_counter += increment 

        return True
    
    ## ------------------------------------  
    # -- Tester Control ---
    ## ------------------------------------        


    
    def BuhnaIsOpen(self):
        # buhna is open
        self.Print('Open Buhna ...')
        return True     
    
    
    

    ## ------------------------------------  
    # -- IO Control ---
    ## ------------------------------------ 

            
    def Status(self):
        self.GetInputStatus()
        self.tprint(f'Checking status')
        
    def Reset(self):
        # get specific bit info
        addr = 0
        self.ResetOutput(addr)
        self.Print(f'IO reset {addr}') 
        
    def GetInfo(self):
        # get specific bit info
        val = self.GetInputStatus(0)
        self.Print(f'IO received {val}')
        
    def SetInfo(self):
        # get specific bit info  
        addr = 0
        val = 1
        self.ioc.SetOutput(addr,val)
        self.Print(f'IO sending value {val} to {addr}')
        
    def CheckAirSupply(self):
        "check if air is in"
        
        # set output to move backward
        #self.SetOutput(1, '00')   
        return True
    
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
        
        ret1 = self.GetInput(1)    
        ret2 = self.GetInput(2) 
        
        self.Print('Preassed %d and %d' %(ret1, ret2))
        
        ret    = ret1 and ret2
        return ret    
        
    def Disconnect(self):
        # disonnecteing
        self.CloseConnectionWithController()  
        
        
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
                self.Print('Move Backward Timeout', 'E')
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
                self.Print('Move Backward Timeout', 'E')
                break
            
        return ret          
            
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
            
    def Home(self):
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
        self.Connect()
        
        # turn on relay 1 with no delay
        self.rStatus = self.SetOutput(1, '00')
        self.Print('output 1:', self.rStatus)
        
        # turn off relay 1 with no delay
        self.rStatus = self.ResetOutput(1)
        self.Print('output 1:', self.rStatus)
        
        # turn on relay 1 with delay of 5 sec
        self.rStatus = self.SetOutput(1, '05')
        self.Print('output 1:', self.rStatus)
        
        # get input 1 
        self.rStatus = self.GetInputStatus(1)
        self.Print('input 1:', self.rStatus)             
        
        self.CloseConnectionWithController()        
        
if __name__ == '__main__':
    c = IOController()
    c.Test()
        
        

