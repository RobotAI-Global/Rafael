import tkinter as tk
import threading
import datetime
import time

from ComClient import ComClient

c_ComClient  = ComClient()
c = c_ComClient

global sHost 
global sPort 

global connected

# Define a function to be executed in a separate thread
def thread_1(): 
    global sHost 
    global sPort
    
    global connected            
    
    c.host = str(sHost)
    c.port = int(sPort)          
    
    # if no connection try to connect     
    while True:        
        try:            
            ConnectionStatus = c.ConnectCleint(sHost, sPort) 
            time.sleep(1)
            
            if ConnectionStatus == 0:
                print('Client is connected')
                
                connected = True
                
                TimeTitel = 'Client connected Time: '
                ShowMessage = str(TimeTitel) + str(getCurrentTime())
                LstData.insert(tk.END, ShowMessage)                
                
                BtnSendData.config(state="normal")
                BtnConnect.config(text="Disconnect")            
            
            #long-running task here - only if connection is open
            while connected == True:                 
                st_Data = c.ReciveData() 
                    
                st_Data = c.msgPacket.print_data('Recived')
                LstData.insert(tk.END, st_Data)
            
            CloseSocket()
                    
        except Exception as e:
            connected = False
            
            TimeTitel = 'Socket Error!!! Client disconnected Time: '
            ShowMessage = str(TimeTitel) + str(getCurrentTime())
            LstData.insert(tk.END, ShowMessage)
            
            BtnSendData.config(state="disable")
            BtnConnect.config(text="Connect")
            
            print(e)

def getCurrentTime():
    lnow = datetime.datetime.now()
    lCrrTime = lnow.strftime("%H:%M:%S") 

    return lCrrTime 

def SendData_Click(): 
    cCommand = TxtCommand.get()
    
    c.SendData(str(cCommand))
    
    st_Data = c.msgPacket.print_data('Send')
    LstData.insert(tk.END, st_Data)

def Connect_click(): 
    global sHost 
    global sPort
    
    sHost = TxtIpAddress.get()
    sPort = TxtPortNumber.get()
    
    # Start a new thread when connect button is clicked
    thread1 = threading.Thread(target=thread_1)
    thread1.start()
    
def Disconnect_Click(): 
    global connected     
    
    TimeTitel = 'Client disconnected Time: '
    ShowMessage = str(TimeTitel) + str(getCurrentTime())
    LstData.insert(tk.END, ShowMessage)
    
    connected = False
    
    BtnSendData.config(state="disable")
    BtnConnect.config(text="Connect")
       
    c.CloseConnection()
    
def CloseSocket():   
    print('Client connection is not availabl')         
    
    Disconnect_Click()         

def toggle_connection():    
    if connected:        
        Disconnect_Click()
        BtnConnect.config(text="Connect")
    else:        
        Connect_click()
        BtnConnect.config(text="Disconnect") 
        
def Delete_List():
    LstData.delete(0, tk.END)

# when form load    
def on_load():
    global connected        
    
    connected = False    
    BtnSendData.config(state="disabled")    
   
# when form unload
def on_unload():
    print("Form unloaded")
    
def exit_application():    
    root.destroy()

# Create the main window
root = tk.Tk()

root.geometry('750x600')
root.title("Client connection")

LblHeader = tk.Label(root, text="Client Form",width=20,font=("bold", 20))
LblHeader.place(x=210,y=53)

LblIpAddress = tk.Label(root, text="IP Address",width=20,font=("bold", 10))
LblIpAddress.place(x=1,y=130)

TxtIpAddress = tk.Entry(root)
TxtIpAddress.place(x=150,y=130)
defaultAddValue = '127.0.0.1'
TxtIpAddress.insert(0, defaultAddValue)

LblCommand = tk.Label(root, text="Command",width=20,font=("bold", 10))
LblCommand.place(x=250,y=130)

TxtCommand = tk.Entry(root)
TxtCommand.place(x=400,y=130)
defaultCommand = '6'
TxtCommand.insert(0, defaultCommand)

LblPortNumber = tk.Label(root, text="Port Number",width=20,font=("bold", 10))
LblPortNumber.place(x=1,y=180)

TxtPortNumber = tk.Entry(root)
TxtPortNumber.place(x=150,y=180)
defaultPortValue = '5000'
TxtPortNumber.insert(0, defaultPortValue)

LblData = tk.Label(root, text="Data Send / Recived",width=20,font=("bold", 10))
LblData.place(x=1,y=230)

# create the list box
LstData = tk.Listbox(root, height=10, width=77, font=("Arial", 12),
                             selectmode=tk.MULTIPLE, background="white",
                             foreground="black", borderwidth=2)
LstData.place(x=20,y=260)

# create a scrollbar and attach it to the list box
scrollbar = tk.Scrollbar(root)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
LstData.config(yscrollcommand=scrollbar.set)
scrollbar.config(command=LstData.yview)

BtnConnect = tk.Button(root, height=2, width=10, text="Delete list",
                       font=("Arial", 12), command = Delete_List)
BtnConnect.place(x=620,y=460)

BtnConnect = tk.Button(root, height=2, width=10, text="Connect",
                       font=("Arial", 12), command = toggle_connection)
BtnConnect.place(x=20,y=530)

BtnSendData = tk.Button(root, height=2, width=10, text="Send Data",
                        font=("Arial", 12), command = SendData_Click)
BtnSendData.place(x=160,y=530)

BtnExit = tk.Button(root, height=2, width=10, text="Exit",
                    font=("Arial", 12), command = exit_application)
BtnExit.place(x=620,y=530)

# Bind on_load function to the <Map> event
root.bind("<Map>", lambda event: on_load())

# Bind on_unload function to the <Unmap> event
root.bind("<Unmap>", lambda event: on_unload())

print('Click the connect button to connect Client with the Server....')

# Start the main event loop
root.mainloop()