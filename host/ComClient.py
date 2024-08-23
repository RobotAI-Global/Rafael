from host.Packet import Packet
import socket
from queue import Queue
from threading import Thread
import time

class ComClient:
    def __init__(self, parent=None, host = '127.0.0.1', port = 5000):
        
        self.parent         = parent 
        self.msgPacket      = Packet()
        self.stopTask       = False
        self.ts             = None
        
        # comm  with the task
        self.queueToHost    = Queue()
        self.queueFromHost  = Queue()          
        
        # initiate the hostname (IP Number)
        # as both code is running on same pc otherwise enter PC IP address
        #self.host = socket.gethostname()
        self.host           = host       
        
        # initiate port no above 1024
        #self.port = 22  # initiate port no above 1024
        self.port           = port  # initiate port no above 1024
        
    def Init(self):
        "common interface"
        self.stopTask       = False 
        while not self.queueToHost.empty():
            self.queueToHost.get()
        while not self.queueFromHost.empty():
            self.queueFromHost.get()    
            
        self.Print('Init done')        
        
    def ConnectCleint(self, chost = None, cport = None):            
        # instantiate
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        chost = self.host if chost is None else chost
        cport = self.port if cport is None else cport
        
        # connect to the server, if result = 0 connection is OK
        result = self.client_socket.connect_ex((str(chost), int(cport)))
        self.Print('Connected %s' %str(result))      
        return result
        
    def ReciveData(self):
        # recive data from the server
        self.reciveData = self.client_socket.recv(1024)         
        #print('Rx-binary : ', self.reciveData)
        
        # extract recived data and get maessages
        self.Print('Recived : ')
        self.msgPacket.packet_recv(self.reciveData)
        
        return self.reciveData
        
    def SendData(self, cCommand = '6'):
        # prepar data to be send to the server
       
               
        self.Print('Transmit : ')
        self.msgPacket.CleintDataToSend(cCommand)
        self.dataBinary =  self.msgPacket.packet_send()          	
        #print('Tx-binary : ', self.dataBinary)
        
        # send data to the server
        self.client_socket.sendall(self.dataBinary)
        
    def ClientThread(self):  
        "main thread"
        try:
            self.ConnectCleint()  
            self.stopTask = False
            self.Print('Client is running')
        except Exception as e:
            print(e)             
                        
        while not self.stopTask:
            
            if self.queueFromHost.empty():
                continue  
            
            cCommand = self.queueFromHost.get()
            self.SendData(cCommand)
            
            time.sleep(0.5)
            
            cData = self.ReciveData()
            
            if not self.queueToHost.empty():
                self.Print('Host must read previous data. Data Loss')
                continue 
            else:
                self.queueToHost.put(cData)
         
        self.Print('Client is finished')
 
                
    def Start(self):
        "running in the thread"
        self.ts = Thread(target = self.ClientThread)
        self.ts.start()     
        
    def Stop(self):
        self.stopTask = True 
        self.Print('Stop')  
        
    def CloseConnection(self):
        # close the connection
        self.client_socket.close() 
        self.Print('closed')
        
    def Print(self, txt):
        print('I: CLI: %s' %str(txt))
        
    def TestHostSimulator(self, msgList = []):
        "simulates host commands"
        self.Init()        
        self.Start()

        msgList = ['2', '3', '1'] if len(msgList) < 1 else msgList
        
        for command in msgList:
            self.Print('Sending %s....' %str(command))
            self.queueFromHost.put(command)
            time.sleep(3)
            self.Print('Receiving....')
            try:
                data = self.queueToHost.put(10)
            except: # timeout
                data = 'no data from Main'
                    
            print(data)
            time.sleep(3)
        
if __name__ == '__main__':
    c = ComClient()
    #c.StartCom()
        