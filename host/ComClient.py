from Packet import Packet
import socket

class ComClient:
    def __init__(self):
        
        self.msgPacket = Packet()
        
        # initiate the hostname (IP Number)
        # as both code is running on same pc otherwise enter PC IP address
        #self.host = socket.gethostname()
        self.host = '127.0.0.1'         
        
        # initiate port no above 1024
        #self.port = 22  # initiate port no above 1024
        self.port = 5000  # initiate port no above 1024
        
    def ConnectCleint(self, chost, cport):            
        # instantiate
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # connect to the server, if result = 0 connection is OK
        result = self.client_socket.connect_ex((str(chost), int(cport)))
                
        return result
        
    def ReciveData(self):
        # recive data from the server
        self.reciveData = self.client_socket.recv(1024)         
        #print('Rx-binary : ', self.reciveData)
        
        # extract recived data and get maessages
        print('Recived : ')
        self.msgPacket.packet_recv(self.reciveData)
        
        return self.reciveData
        
    def SendData(self, cCommand):
        # prepar data to be send to the server
        print('Transmite : ')
        self.msgPacket.CleintDataToSend(cCommand)
        self.dataBinary =  self.msgPacket.packet_send()          	
        #print('Tx-binary : ', self.dataBinary)
        
        # send data to the server
        self.client_socket.sendall(self.dataBinary)
        
    def CloseConnection(self):
        # close the connection
        self.client_socket.close()    
        
if __name__ == '__main__':
    c = ComClient()
    #c.StartCom()
        