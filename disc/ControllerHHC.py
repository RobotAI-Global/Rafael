"""

RobotAI : IO Interface

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

import socket
#from queue import Queue
from threading import Thread
import time

try:
    from gui.Logger import logger
    print('main app')
except:
    import logging
    logger      = logging.getLogger("robotai")
    print('local debug')

#%%
class IOController:
    def __init__(self, parent=None, host = '192.168.2.105', port = 5000):
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
        logger.info('Init')
        
        
    def Connect(self):
        # Create a TCP/IP socket
        #if not self.IsConnected():
        self.client_socket  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # connect to the server, if result = 0 connection is OK
        result              = self.client_socket.connect((self.ServerIp, self.ServerPort))
        logger.info('Connected to the controller.')
        # else:
        #     result              = True
        #     logger.info('Reusing previous connection.')
        
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
    
    def Default(self):
        # set default values
        ret = True
        return ret      
        
    def SendDataToController(self, cCommand):
        # Send data to the server       
        self.client_socket.sendall(cCommand.encode())
        #logger.info('Message sent to the controller:', cCommand)        
    
    def ReciveDataFromController(self, cCommand, cNumber):
        # Receive data from the server
        self.data = self.client_socket.recv(1024).decode()
        #logger.info('Message received from the controller:', self.data)       

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
        res = self.ReciveDataFromController('input', iNumber) # self.rValue
        #logger.info('GetInputStatus : %s' %str(self.rValue))
        
        logger.debug(f'Input {iNumber} : {res}')
        return res
    
    def SetOutput(self, iNumber, Cmnd = 'on', sDelay = '00'):
        self.sCommand       = Cmnd
        self.OutputNumber   = str(iNumber)
        
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
        logger.debug('SetOutput %s : %s' %(str(iNumber),str(self.rValue)))
        
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
        
        logger.debug('ResetOutput : %s' %str(iNumber))
        return self.rValue
        
    def CloseConnectionWithController(self):
        # close the connection
        self.client_socket.close()
        logger.info('Connection with the controller closed.')
        
    def Close(self):
        "dublicated"
        self.CloseConnectionWithController()
        
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
        logger.info('Start')    
        self.RunThread()
        
    def Stop(self):
        logger.info('Stop') 
        self.stopTask = True
        self.ts.join()          
    
#    ## ------------------------------------  
#    # -- Table Control ---
#    ## ------------------------------------        
#    def SetTableHome(self):
#        # go to home position
#        logger.info('Starting Homing ...')
#        
#        home_sensor = False
#        while not home_sensor:
#             
#            # read home sensor
#            home_sensor = True
#        
#        return True
#        
#    def GetTableIndex(self):
#        # go to home position
#        logger.info('Current table position ...')
#        return True
#    
#    def NextTableIndex(self, increment = 1):
#        # go to next position
#        logger.info('Next table position ...')
#        
#        # two stations
#        self.MoveTableIndex()
#        self.MoveTableIndex()
#  
#        self.uut_counter += increment 
#
#        return True
    


    ## ------------------------------------  
    # -- IO Control ---
    ## ------------------------------------ 

            
    def Status(self):
        self.GetInputStatus()
        logger.info(f'Checking status')
        
    def Reset(self):
        # get specific bit info
        addr = 0
        self.ResetOutput(addr)
        logger.info(f'IO reset {addr}') 
        
    def GetInfo(self):
        # get specific bit info
        val = self.GetInputStatus(0)
        logger.info(f'IO received {val}')
        
    def SetInfo(self):
        # get specific bit info  
        addr = 0
        val = 1
        self.ioc.SetOutput(addr,val)
        logger.info(f'IO sending value {val} to {addr}')
        
#    def CheckAirSupply(self):
#        "check if air is in"
#        
#        # set output to move backward
#        #self.SetOutput(1, '00')   
#        return True
#    
#    def CheckEmergencyStop(self):
#        "check if emergency is pressed"
#        
#        # set output to move backward
#        #self.SetOutput(1, '00')   
#        return True 
    

 
        
    def Test(self):
        self.Connect()
        
        # turn on relay 1 with no delay
        self.rStatus = self.SetOutput(1, '00')
        logger.info('output 1:', self.rStatus)
        
        # turn off relay 1 with no delay
        self.rStatus = self.ResetOutput(1)
        logger.info('output 1:', self.rStatus)
        
        # turn on relay 1 with delay of 5 sec
        self.rStatus = self.SetOutput(1, '05')
        logger.info('output 1:', self.rStatus)
        
        # get input 1 
        self.rStatus = self.GetInputStatus(1)
        logger.info('input 1:', self.rStatus)             
        
        self.Close()   
        
    def TestScan(self):
        "scanning all inputs"
        self.Connect()
        for k in range(1,9):
            res = self.GetInputStatus(k)
            
        self.Close()
               
            
        
if __name__ == '__main__':
    c = IOController()
    #c.Test()
    c.TestScan()
        
        

