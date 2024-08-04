"""
 Implements communication to the host

"""

from Packet import Packet
import socket
from queue import Queue
from threading import Thread
import time

class ComServer:
    
    def __init__(self, parent=None):
        #super().__init__()
        self.parent         = parent        
        self.msgPacket      = Packet()        
        self.queueToHost    = Queue()
        self.queueFromHost  = Queue()        
        
        # get the hostname (IP Number)
        # as both code is running on same pc
        #self.host = socket.gethostname()        
        self.host = '127.0.0.1'
        
        # initiate port no above 1024
        #self.port = 22  # initiate port no above 1024
        self.port = 5000  # initiate port no above 1024
        self.stopTask       = False 
        self.ts             = None
        
    def Init(self):
        "common interface"
        self.stopTask       = False 
        while not self.queueToHost.empty():
            self.queueToHost.get()
        while not self.queueFromHost.empty():
            self.queueFromHost.get()    
            
        print('Init done')
        
    def ConnectServer(self):
        # get instance
        self.server_socket = socket.socket() 
        
        # bind host address and port together
        #the bind() function takes tuple as argument
        self.server_socket.bind((self.host, self.port))  
        print("Waiting for connection with the Server....")
        
        # configure how many client the server can listen simultaneously
        self.server_socket.listen(2)
        
        # accept new connection
        self.conn, address = self.server_socket.accept()
        print("Server connected from: " + str(address))        
        
    def ReciveData(self):
        # receive data stream. it won't accept data packet greater than 1024 bytes
        self.reciveData = self.conn.recv(1024)
        #print('Rx :', self.reciveData)
        
        if not self.reciveData:
            print('No Data')    
        else:
            # extract recived data and get maessages
            print('Recived : ')
            self.msgPacket.packet_recv(self.reciveData)        
            #print("Recived: ", msgPacket.msgCount) # print keep a life counter
            
            self.queueFromHost.put(self.msgPacket)
        
    def SendData(self):     
        "send data "
        if self.queueToHost.empty():
            return
        
        self.msgPacket = self.queueToHost.get()
        
        # prepar data to be send to the client
        print('Transmite : ')
        self.dataBinary = self.msgPacket.packet_send()          	
        #print('Tx :',self.dataBinary)
        
        # send data to the client                
        self.conn.sendall(self.dataBinary)
        
    def ServerThread(self):  
        "main thread"
        
        while not self.stopTask:
            try:
                self.ConnectServer()    
                print('Server is running')
                while True:
                    #                     
                    self.ReciveData()
                    time.sleep(0.5)
                    self.SendData()
                    
            except Exception as e:
                print(e)  
                
    def RunThread(self):
        "running in the thread"
        self.ts = Thread(target = self.ServerThread)
        self.ts.start()
        self.ts.join()    
        return True             
        
    def CloseConnection(self):
        # close the connection
        self.conn.close()                                       
        
if __name__ == '__main__':
    s = ComServer()
    s.RunThread()
    
    
    """
    ## ------------------------------------  
    # -- Host Control ---
    ## ------------------------------------ 



    def hostConnect(self):
        # start
        self.tprint('Starting host connection ...')
        
        # maybe already running
        if self.vis is None:
            #print(self.vis)
            # runs Robot server and Multi Object Detection
            self.vis    = HostManager(host = self.ip_vision, port = self.port_vision, debug = self.debugOn, config=self.cfg) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
            self.vis.start()
        elif self.vis.isAlive():   
            self.tprint('Pose6D Connection is alive')
        else:
            #self.vis.start()
            self.tprint('Vision Connection is restarted')
            
    def hostStop(self):
        # stop
        self.tprint('Stop host  ...')
        self.vis.stop()
            
    def hostStatus(self):
        # start
        self.tprint('Getting host status ...')


    def hostDisConnect(self):
        # start
        self.tprint('Disconnect from host ...')
        
        if self.sim is not None:  
            self.sim.stop()       
            
        # maybe already running
        if self.vis is not None:
            self.vis.stop()
            self.tprint('Stopping host thread...')         
            
    """