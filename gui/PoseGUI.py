# -*- coding: utf-8 -*-
"""
Controls pose

@author: zion
"""
import tkinter as tk
#import tkinter.ttk as ttk
#style.use("dark_background")

import os
base_path               = os.path.dirname(os.path.abspath(__file__))
iconFilePath            = os.path.join(base_path, 'logo.ico')

class PoseGUI:
    # constructor method
    def __init__(self, pose = [0,0,0,0,0,0]):
        
        if len(pose) != 6:
            pose = [0,0,0,0,0,0]
            
        self.pose   = pose
 
        # chat window which is currently hidden
        self.root   = tk.Toplevel() #tk.Tk()
        self.root.title("Pose Information")
        self.root.geometry("300x600")
        self.root.iconbitmap(iconFilePath)
        #self.root.withdraw()
        
        self.tx_label = tk.Label(self.root, text="Tx [mm] (Ex: -192.24):", font=("Arial", 10))
        self.tx_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.tx_entry = tk.Entry(self.root)
        self.tx_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.tx_entry.focus()
        
        self.ty_label = tk.Label(self.root, text="Ty [mm] (Ex: 105.26):", font=("Arial", 10))
        self.ty_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.ty_entry = tk.Entry(self.root)
        self.ty_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.ty_entry.focus()
        
        self.tz_label = tk.Label(self.root, text="Tz [mm] (Ex: 255.0):", font=("Arial", 10))
        self.tz_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.tz_entry = tk.Entry(self.root)
        self.tz_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.tz_entry.focus()
        
        self.rx_label = tk.Label(self.root, text="Rx [deg] (-180:180):", font=("Arial", 10))
        self.rx_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.rx_entry = tk.Entry(self.root)
        self.rx_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.rx_entry.focus()
        
        self.ry_label = tk.Label(self.root, text="Ry [deg] (-180:180):", font=("Arial", 10))
        self.ry_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.ry_entry = tk.Entry(self.root)
        self.ry_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.ry_entry.focus()
        
        self.rz_label = tk.Label(self.root, text="Rz [deg] (-180:180):", font=("Arial", 10))
        self.rz_label.pack(padx=10, pady=10, anchor=tk.W)        
        self.rz_entry = tk.Entry(self.root)
        self.rz_entry.pack(padx=10, pady=10, anchor=tk.W, fill = 'x')
        self.rz_entry.focus()        
        
        self.save_button = tk.Button(self.root, text="Save", command= self.save_callback , font=("Arial", 10))
        self.save_button.pack(padx=10, pady=10, side = 'left') #anchor=tk.W)   
        
        self.cancel_button = tk.Button(self.root, text="Cancel", command= self.cancel_callback , font=("Arial", 10))
        self.cancel_button.pack(padx=10, pady=10, side = 'right') #anchor=tk.W)         

        self.init_gui()

        self.root.mainloop()
        
    def init_gui(self):
        self.tx_entry.delete(0,tk.END) 
        self.tx_entry.insert(0,self.pose[0]) 
        self.ty_entry.delete(0,tk.END) 
        self.ty_entry.insert(0,self.pose[1])         
        self.tz_entry.delete(0,tk.END) 
        self.tz_entry.insert(0,self.pose[2])                 
        self.rx_entry.delete(0,tk.END) 
        self.rx_entry.insert(0,self.pose[3]) 
        self.ry_entry.delete(0,tk.END) 
        self.ry_entry.insert(0,self.pose[4])         
        self.rz_entry.delete(0,tk.END) 
        self.rz_entry.insert(0,self.pose[5]) 
        
    def finish_gui(self):
        self.root.destroy()
        self.root.quit()
        
    def read_value(self, entry , checkRange = False):
        val_txt       = entry.get()
        try:
            val_float    = float(val_txt)
        except:
            val_float    = 0.0
            print('Need to be a float number')
            
        if not checkRange:
            return val_float
        
        if val_float < -180 : 
            val_float = -180
        if val_float > 180 : 
            val_float = 180            
                        
        return val_float
        
    def save_callback(self):
        
        self.pose[0]    = self.read_value(self.tx_entry)
        self.pose[1]    = self.read_value(self.ty_entry)
        self.pose[2]    = self.read_value(self.tz_entry)  
        self.pose[3]    = self.read_value(self.rx_entry, True)
        self.pose[4]    = self.read_value(self.ry_entry, True)
        self.pose[5]    = self.read_value(self.rz_entry, True)         
        
        print('save is succesful')
        self.finish_gui()
        return True
        
    def cancel_callback(self):       
        print('cancel is pressed')  
        self.finish_gui()
