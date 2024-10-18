import struct
import datetime

#%% Logger
import logging
logger      = logging.getLogger("robot")


#%% Main

class Packet:

    def __init__(self):    

        self.debugOn        = True            
    
        self.wsync          = 2863289685
        self.msgSize        = 0
        self.cmdCode        = 51
        self.msgCount       = 0
        
        self.data           = [] 
        
        # opcode 51
        self.manual_auto    = 0
        self.UUT_type       = 1
        self.stand_number   = 0
        self.command        = 0 #7
        
        # opcode 52
        self.bit_status     = 0
        self.seconds        = 0
        self.error_codes    = [0]*10
                
        # opcode 53
        self.last_cmd_status = 0
        self.general_status = [0]*10         
        
    def extract_header(self, binaryData = b''):
        if len(binaryData) < 16:
            self.tprint("No header received")
            return False 
               
        data = struct.unpack('<LLLL', binaryData[0:16])
        #print("Data recived in extract header fun: ", data)               
        
        self.wsync, self.msgSize, self.cmdCode, self.msgCount = data         
        self.tprint("Header extract: ", self.wsync, self.msgSize, self.cmdCode, self.msgCount)        
               
        return True
    
    def create_header(self):
        data = (self.wsync, self.msgSize, self.cmdCode, self.msgCount)
        self.tprint("Creating header: ", data) 

        dataBinary = struct.pack("<LLLL", data[0],data[1],data[2],data[3])           	
  
        return dataBinary    
    
    def extract_data(self, binaryData = b''):
        # extract data
        if self.cmdCode == 50:
            self.tprint('OpCode 50 Not supported')
            return False
        
        elif self.cmdCode == 51:
            data = struct.unpack('<LLLL', binaryData[16:32])   
            self.manual_auto, self.UUT_type, self.stand_number, self.command = data
            
            self.tprint('Data extract OpCode 51: ', self.manual_auto, self.UUT_type, self.stand_number, self.command)
            
        elif self.cmdCode == 52:
            data = struct.unpack('<LL', binaryData[16:24])   
            self.bit_status, self.seconds = data
            
            data = struct.unpack('<LLLLLLLLLL', binaryData[24:65])
            self.error_codes = data 
            
            self.tprint('Data extract OpCode 52: ', self.bit_status, self.seconds,  self.error_codes)
        
        elif self.cmdCode == 53:
            data = struct.unpack('<L', binaryData[16:20])
            self.last_cmd_status = data
            
            data = struct.unpack('<LLLLLLLLLL', binaryData[20:60])
            self.general_status = data 
            
            self.tprint('Data extract OpCode 53: ', self.last_cmd_status, self.general_status)    
                    
        else:
            
            self.tprint('bad cmdCode to extract data')
            return False
        
        return True
    
    def create_data(self):
        # pack data
        dataBinary = b''
        if self.cmdCode == 50:
            self.tprint('OpCode 50 Not supported')
            return False
            
        elif self.cmdCode == 51:             
            data = (self.manual_auto, self.UUT_type, self.stand_number, self.command)            
            dataBinary = struct.pack("<LLLL", data[0],data[1],data[2],data[3])
            self.tprint('Create date OpCode 51: ',self.manual_auto, self.UUT_type, self.stand_number, self.command)
            #print('Create binary date: ', dataBinary)
            
        elif self.cmdCode == 52:            
            data = (self.bit_status, self.seconds) 
            dataBinary_1 = struct.pack("<LL", data[0],data[1])
            
            data = self.error_codes[0:4]
            dataBinary_2 = struct.pack("<LLLL", data[0],data[1],data[2],data[3])
            data = self.error_codes[4:8]
            dataBinary_3 = struct.pack("<LLLL", data[0],data[1],data[2],data[3]) 
            data = self.error_codes[8:10]
            dataBinary_4 = struct.pack("<LL", data[0],data[1])           
            
            dataBinary = dataBinary_1 + dataBinary_2 + dataBinary_3 + dataBinary_4
            self.tprint('Create date OpCode 52:',self.bit_status, self.seconds, self.error_codes)
            #print('Create binary date: ', dataBinary)             
            
        elif self.cmdCode == 53:
            data = (self.last_cmd_status) 
            dataBinary_1 = struct.pack("<L", data)
            
            data = self.general_status[0:4]
            dataBinary_2 = struct.pack("<LLLL", data[0],data[1],data[2],data[3])
            data = self.general_status[4:8]
            dataBinary_3 = struct.pack("<LLLL", data[0],data[1],data[2],data[3]) 
            data = self.general_status[8:10]
            dataBinary_4 = struct.pack("<LL", data[0],data[1])           
            
            dataBinary = dataBinary_1 + dataBinary_2 + dataBinary_3 + dataBinary_4
            self.tprint('Create date OpCode 53:', self.last_cmd_status, self.general_status)
            #print('Create binary date: ', dataBinary)                            
                
        else:
            self.tprint('bad cmdCode to create data')
            return False
        
        return dataBinary 
    
    def print_data(self, command):
        # print recived date on screen
        # first prind Header
        if command == 'Send':
            titel = "Send Header: "
        elif command == 'Recived':
           titel = "Recived Header: " 
           
        #dt_Header = str(self.wsync), str(self.msgSize), str(self.cmdCode), str(self.msgCount)        
        dt_Header = self.wsync, self.msgSize, self.cmdCode, self.msgCount
        
        if self.cmdCode == 51:
            dt_titel = " Data: No data"             
            all_Data = str(titel) + str(dt_Header) + str(dt_titel)
        
        elif self.cmdCode == 52:
            dt_titel = " Data: "
            dt_Data = self.bit_status, self.seconds 
            dt_ErrWr = (self.error_codes[0], self.error_codes[1], self.error_codes[2],
                        self.error_codes[3], self.error_codes[4], self.error_codes[5],
                        self.error_codes[6], self.error_codes[7], self.error_codes[8],
                        self.error_codes[9])            
            
            all_Data = str(titel) + str(dt_Header) + str(dt_titel) + str(dt_Data) + str(dt_ErrWr)
            
        elif self.cmdCode == 53:
            dt_titel = " Data: "
            dt_Data = self.last_cmd_status
            dt_Status = (self.general_status[0], self.general_status[1],
                         self.general_status[2], self.general_status[3], 
                         self.general_status[4], self.general_status[5],
                         self.general_status[6], self.general_status[7], 
                         self.general_status[8], self.general_status[9])            
            
            all_Data = str(titel) + str(dt_Header) + str(dt_titel) + str(dt_Data) + str(dt_Status)
        
        dt_Time = ' Time: ' 
        # get current time
        now = datetime.datetime.now()
        CurrentTime = now.strftime("%H:%M:%S")        
        all_Data = all_Data + dt_Time + str(CurrentTime)
        
        return all_Data
    
    def CleintDataToSend(self, cCommand):
        self.wsync, self.msgSize, self.cmdCode, self.msgCount = 2863289685, 32, 51, 1
        self.manual_auto, self.UUT_type, self.stand_number, self.command = 0, 1, 0, int(cCommand)
        
    def packet_recv(self, binaryData = b''):
        # entire packet decode
        isOk = self.extract_header(binaryData)
        if not isOk:
            return False
        
        isOk = self.extract_data(binaryData)
        if not isOk:
            return False
        
    def packet_send(self):
        # entire packet encode
        binaryHeader = self.create_header()
        #if not isOk:
        #    return False
        
        binaryData = self.create_data()
        #if not isOk:
        #    return False 
        
        binaryData = binaryHeader + binaryData
        
        return binaryData  

    def tprint(self, *args, **kwargs):
        if not self.debugOn:
            return        
        newstr = ""
        for a in args:
            newstr+=str(a)+' '        
        #print('I: PKT: %s' %str(txt))
        logger.info(newstr)
    
    # ****** Subroutines to test the packet ******
    def test_header(self):
        self.tprint('testing header')
        self.wsync, self.msgSize, self.cmdCode, self.msgCount = 2863289685, 32, 52, 1
        
        binHeader = self.create_header()
        self.extract_header(binHeader)
        
        print(self.wsync, self.msgSize, self.cmdCode, self.msgCount)
        
    def test_data(self):
         self.tprint('testing data')
         self.wsync, self.msgSize, self.cmdCode, self.msgCount = 2863289685, 32, 52, 1
         self.manual_auto, self.UUT_type, self.stand_number, self.command = 0, 1, 0, 1
         
         binHeader = self.create_data()
         self.extract_data(binHeader + binHeader)
         
         print(self.manual_auto, self.UUT_type, self.stand_number, self.command)       
        
    def test_packet(self):
         self.tprint('testing packet')
         
         binHeaderData = b'UU\xaa\xaa \x00\x00\x003\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00'         
         print('Recived data from client: ', binHeaderData)
         
         self.packet_recv(binHeaderData)         
         print('Data: ', self.manual_auto, self.UUT_type, self.stand_number, self.command) 
         
         self.wsync, self.msgSize, self.cmdCode, self.msgCount = 2863289685, 32, 52, 1
         self.manual_auto, self.UUT_type, self.stand_number, self.command = 0, 1, 0, 1
         
         binHeaderData = self.packet_send()
         print('Sending data from server: ', binHeaderData)    

if __name__ == '__main__':
    p = Packet()
    #p.test_header()  
    #p.test_data()
    p.test_packet()
    
    
    
    