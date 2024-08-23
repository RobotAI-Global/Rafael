"""
 Implements communication to the host

"""

from host.Packet import Packet
import socket
from queue import Queue
from threading import Thread
import time


#%% Logger
import logging
logger      = logging.getLogger("robot")


#%% Main

class ComServer:
    
    def __init__(self, parent=None, host = '127.0.0.1', port = 5000):
        #super().__init__()
        self.parent         = parent        
        self.msgPacket      = Packet()        
        self.queueToHost    = Queue()
        self.queueFromHost  = Queue()     
        self.server_socket  = None
        self.conn           = None
        
        # get the hostname (IP Number)
        # as both code is running on same pc
        #self.host = socket.gethostname()        
        self.host 			= host #'127.0.0.1'
        
        # initiate port no above 1024
        #self.port = 22  # initiate port no above 1024
        self.port 			= port  # initiate port no above 1024
        self.stopTask       = False 
        self.ts             = None
        
    def Init(self):
        "common interface"
        self.stopTask       = False 
        while not self.queueToHost.empty():
            self.queueToHost.get()
        while not self.queueFromHost.empty():
            self.queueFromHost.get() 
            
        self.Print('Init done')
        
    def ServerStart(self):
        # get instance
        if self.server_socket is None:
            self.server_socket = socket.socket() 
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #self.server_socket.settimeout(timeout_server)
            
            # bind host address and port together
            #the bind() function takes tuple as argument
            self.server_socket.bind((self.host, self.port))  
        else:
            self.Print('Old connection exists')
            
        self.Print("Waiting for connection with the Server....")  
        
    def ServerClose(self):
        # close the connection
        self.server_socket.close()  
        self.Print('Closed')        
        
    def ClientConnect(self):
        "waititing for client"

        self.Print('Server is waiting for client')
        # configure how many client the server can listen simultaneously
        self.server_socket.listen()
        
        # accept new connection
        self.conn, address = self.server_socket.accept()
        self.Print("Server connected from: " + str(address))      
        
    def ClientClose(self):
        # close the connection
        self.conn.close()  
        self.Print('Closed')        
        
    def ReciveData(self):
        
        # receive data stream. it won't accept data packet greater than 1024 bytes
        self.reciveData = self.conn.recv(1024)
        #print('Rx :', self.reciveData)
        
        if not self.reciveData:
            self.Print('No Data')    
        else:
            # extract recived data and get maessages
            self.Print('Recived : ')
            self.msgPacket.packet_recv(self.reciveData)        
            #print("Recived: ", msgPacket.msgCount) # print keep a life counter
            
            self.queueFromHost.put(self.msgPacket)
        
    def SendData(self):     
        "send data "
        if self.queueToHost.empty():
            return
        
        self.msgPacket = self.queueToHost.get()
        
        # prepar data to be send to the client
        self.Print('Transmit : ')
        self.dataBinary = self.msgPacket.packet_send()          	
        #print('Tx :',self.dataBinary)
        
        # send data to the client                
        self.conn.sendall(self.dataBinary)
        
    def RecvMessage(self):
        "called to get packet"
        msgFromHost = self.queueFromHost.get()
        return msgFromHost
    
    def SendMessage(self, msgToHost):
        "called to send packet"
        ret = False
        if self.queueToHost.empty():        
            self.queueToHost.put(msgToHost)
            ret  = True
        else:
            self.Print('Busy : Packet Drop')
            
        return ret    
        
    def ServerThread(self):  
        "main thread"
        self.ServerStart()
        self.stopTask = False
        while not self.stopTask:
                        
            self.ClientConnect()            
            while not self.stopTask:
      
                self.ReciveData()
                time.sleep(0.5)
                self.SendData()
                
            self.ClientClose()
                
        self.ServerClose()
        self.Print('Thread is finished')
                
    def Start(self):
        "running in the thread"
        self.ts = Thread(target = self.ServerThread)
        self.ts.start()
        #self.ts.join()    
        #return True    
        
    def Stop(self):
        self.stopTask = True 
        self.Print('Stop')  
        
    def IsConnected(self):
        "indicates if client is connected"
        ret = not(self.conn is None)
        return ret
        

    def Print(self, txt):
        #print('I: SRV: %s' %str(txt))   
        logger.info(txt)                                 
        
if __name__ == '__main__':
    s = ComServer()
    s.Start()
    
