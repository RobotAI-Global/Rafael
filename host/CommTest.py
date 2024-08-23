# -*- coding: utf-8 -*-
"""
Created on Fri Aug 16 08:52:12 2024

@author: zion

Test Comm Client and Server for Host

"""

from host.ComClient import ComClient
from host.ComServer import ComServer
import time


COMM_IP     = '127.0.0.1'
COMM_PORT   = 8480
TIMEOUT     = 10   # sec

#%% Logger
import logging
logger      = logging.getLogger("robot")
#formatter    = logging.Formatter("{levelname} - {message}", style="{")
#formatter   = logging.Formatter('[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s', datefmt="%M:%S", style="{")
formatter   = logging.Formatter('[%(asctime)s] - {%(filename)12s:%(lineno)3d} - %(levelname)s - %(message)s')
logger.setLevel("DEBUG")

console_handler = logging.StreamHandler()
console_handler.setLevel("DEBUG")
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

file_handler = logging.FileHandler("main_app.log", mode="a", encoding="utf-8")
file_handler.setLevel("WARNING")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

       
#%% Tests           
class TestServerClient: #unittest.TestCase
    
    def __init__(self):
        # inint two objects
        self.server = ComServer(parent = None, host = COMM_IP, port = COMM_PORT)
        self.client = ComClient(parent = None, host = COMM_IP, port = COMM_PORT)
        
    def Start(self):
        # start running
        self.server.Init()
        self.server.Start()
        
        time.sleep(1)
        
        self.client.Init()
        self.client.Start() 
        
        time.sleep(1)
        
    def Stop(self):
        # Stop running
        self.client.Stop()         
        time.sleep(1)        
        self.server.Stop()
         
    def TestStartStop(self): 
        "test connect and disconnect"
        self.Print('Cycle 1 ========')
        self.Start()
        time.sleep(1)
        self.Stop()
        self.Print('Cycle 2 ========')
        self.Start()
        time.sleep(1)
        self.Stop()        
        
    def TestOneMessage(self):
        "test message send"
        self.Print('Msg 1 ========')
        self.Start()
        command     = '6'
        self.Print('Sending....')
        self.client.queueFromHost.put(command)
        time.sleep(1)
        self.Print('Receiving....')
        data = self.client.queueToHost.put(10)
        self.Print(data)
        self.Stop()
    
        
    def TestAllMessages(self):
        "test messages to send"
        self.Print('Test starts')
        self.Start()
        
        msgList = ['5', '6', '1'] 
        for command in msgList:
            self.Print('Sending %s....' %str(command))
            self.client.queueFromHost.put(command)
            time.sleep(1)
            self.Print('Receiving....')
            data = self.client.queueToHost.put(10)
            self.Print(data)
            
        self.Stop()
        
    def Print(self,txt):
        logger.info(str(txt))


        
#%%
if __name__ == "__main__":
    """
    You can run TestClient and TestServer in different kernels
    or you can run TestBoth in the same Kernel/Console
    """
    
    tsc = TestServerClient()
    
    #tsc.Start() # ok
    #tsc.TestStartStop() #ok
    #tsc.TestOneMessage()
    tsc.TestAllMessages()
    

