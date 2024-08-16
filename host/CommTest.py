# -*- coding: utf-8 -*-
"""
Created on Fri Aug 16 08:52:12 2024

@author: zion

Test Comm Client and Server for Host

"""

from ComClient import ComClient
from ComServer import ComServer
import time


COMM_IP     = '127.0.0.1'
COMM_PORT   = 8480
TIMEOUT     = 10   # sec

       
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
        print('Cycle 1 ========')
        self.Start()
        time.sleep(1)
        self.Stop()
        print('Cycle 2 ========')
        self.Start()
        time.sleep(1)
        self.Stop()        
        
    def TestOneMessage(self):
        "test message send"
        print('Msg 1 ========')
        self.Start()
        command     = '6'
        print('Sending....')
        self.client.queueFromHost.put(command)
        time.sleep(1)
        print('Receiving....')
        data = self.client.queueToHost.put(10)
        print(data)
        self.Stop()
    
        
    def TestAllMessages(self):
        "test messages to send"
        
        self.Start()
        
        msgList = ['5', '6', '1'] 
        for command in msgList:
            print('Sending %s....' %str(command))
            self.client.queueFromHost.put(command)
            time.sleep(1)
            print('Receiving....')
            data = self.client.queueToHost.put(10)
            print(data)
            
        self.Stop()


        
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
    

