# -*- coding: utf-8 -*-
"""
Configuration File Manager
Usage : 
    from ConfigManager import ConfigManager
    p = ConfigManager()
    p.Test()
    
Other functionality:
    p.Init()  # creates directory and related files
    
    
-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101   14.06.24 UD     init
-----------------------------

"""


#%% Libraries
import numpy as np
#from tkinter import filedialog
import os
#import pickle
#import yaml
import json

from queue import Queue

import unittest
import logging as log


session_json_default = {
    "version" : "0201",
    "robot"   : {
        "position_home" : [-0.689, -0.121, 1.035, -2.0, 0.0, 2.0],
        "position_units": "meter-rad"
    },
    "cnotrol"   : {
        "position_home" : [-0.689, -0.121, 1.035, -2.0, 0.0, 2.0],
        "position_units": "mm-degree"
    },
    "io"   : {
        "position_home" : [-0.689, -0.121, 1.035, -2.0, 0.0, 2.0],
        "position_units": "mm-degree"
    }    
}

#%%---------------------------------------------------------------------------------------------------------------------------------------

class ConfigManager:
        
    def __init__(self, workingDirectory = ''):
                
        self.path           = '' 
        #self.serverMode     = True     # holds the state if the server mode is activated 
        self.debugOn        = True    # global debug
        
#        # define quese to send receive 
#        self.queueToMain    = Queue(1)
#        self.queueFromMain  = Queue(1)
#        self.queueMsg       = {'Id': 0, 'Data' : 0}   # message example

        # denso request to filter addresses in robot server
        #self.client_host    = '127.0.0.1'   # user specified robot client ip
        
        # make it available more easy
        #self.ROBOT_LIST     = ROBOT_LIST
        
        self.Init(workingDirectory)          
        self.Print("ConfigManager Created")
        
    def Init(self, workingDirectory = ''):
        # try to initialize itself
        if not self.SetWorkingDirectory(workingDirectory) :
            return
            
        # check file exists or create new one
        if not self.SetupConfigFile():
            return
        
              
        configFileName = self.GetSessionFileName()
        self.Print("Config file : %s" % configFileName)              

    def SetWorkingDirectory(self, workingDirectory = ''):
        "set project directory"
        # try to recover - may be already initialized
        ret = False
        if len(workingDirectory)<2:
            workingDirectory = self.path            
            
        # true object directory
        if not os.path.isdir(workingDirectory):
            workingDirectory  = os.getcwd()
            self.Print("Bad working directory %s - initializing " % workingDirectory)
        
        self.path = workingDirectory
        ret = True
        return ret  


    def IsFileExist(self):  
        # test if file exists
        ret = True
        fileName = self.GetSessionFileName()
        if not os.path.isfile(fileName):
            self.Print("Can not find: %s" % fileName)
            ret = False
            return 
        #else:
            #self.Print("File found: %s" % fileName)
        return ret  
    
    def GetCameraFolder(self):
        working_directory  = self.path
        return os.path.join(working_directory, 'gui') 
         
    
    def GetVisionFolder(self):
        working_directory  = self.path
        return os.path.join(working_directory, 'vision') 
    
    def GetRobotFolder(self):
        working_directory  = self.path
        return os.path.join(working_directory, 'robot')     

    def GetSessionFileName(self):
        " read object spec files and writes it into config file"
        working_directory  = self.path
        filename           = os.path.join(working_directory,'main_session.json')
        return filename        
   
    def SetupConfigFile(self):
        "setup file in the location"
        ret = False
        if not os.path.isdir(self.path):
            self.Print("Bad working directory %s" % self.path, 'W')
            return ret
        
        # object points and their coordinates
        ret = True
        configFile         = self.GetSessionFileName()
        if os.path.isfile(configFile):
             self.Print("File already exists %s" % configFile, 'I')
             return ret
         
        self.CreateSessionFile()
        return ret
        

    
    def MergeCfgAintoB(self, cfgA, cfgB):  
        # meging to condif files and check parameters

        """Merge config dictionary a into config dictionary b, clobbering the
        options in b whenever they are also specified in a.
        """    
        ret  = True
        cfgO = cfgB
        
        # some strange problem
        if cfgA is None:
            return cfgO, False 
        
        for k, v in cfgA.items():
            # a must specify keys that are in b
            #if k not in b:
            #    raise KeyError('{} is not a valid config key'.format(k))
            try:
#                vold    = cfgB[k]
#                if vold != v:
#                    self.Print('The default vaue of {} is updated'.format(k))
                cfgB[k] = v
            except:
                self.Print('Error in config file, check field: {}'.format(k),'E')
                ret = False
                #break
                
        if ret: 
             cfgO = cfgB
             
        return cfgO, ret    
    
            
    def GetDebugLevel(self):
                
        res           = 51 
        cfg           = self.ReadPlainconfig()   
        if cfg is None:
            return res
            
        if 'net_level' in cfg:
            res = cfg['net_level']
                       
        return res


    
    def GetSessionParameters(self):
        # read certain params
        filename           = self.GetSessionFileName()
        try:
            with open(filename, 'rb') as f:
                data = json.load(f)                         

        except:
            self.Print('Can not read JSON file %s. Using default values' % filename,'E')                
            data = {}
            
        return data    
    


        
    def CreateSessionFile(self, json_data = None):
        " create object spec file and writes it into config file"
     
        ret                = False
        filename           = self.GetSessionFileName()  
        
        if json_data is None:
            json_data       = session_json_default
        
        if os.path.isfile(filename):       
            self.Print('Object json file exists')
            return True
        
        try:
            with open(filename, "w") as outfile: 
                #json.dump(json_data, outfile) 
                json.dump(json_data, outfile, indent=4) 
            ret = True
        except:
            self.Print('Can not write JSON file %s' % filename,'E')                
            return ret
                   
        self.Print('Session json file is created %s' %filename)
        return ret
        
    
    def UpdateSessionFile(self, mtrx, dist):
        " write camera params into a JSON file"

        ret                = False
        filename           = self.GetSessionFileName();
        
        # empty list
        if len(mtrx) < 1:
            return ret
        
        # check inputs
        if len(mtrx.ravel()) != 9:
            self.Print('Error bad camera matrix','E')
            return ret
        
        if len(dist.ravel()) < 5:
            self.Print('Error bad camera distortion matrix','E')
            return ret
        
        
        # try to read json first
        try:
            with open(filename, 'rb') as f:
                data = json.load(f)
                
            coord = data.get('version')
            if coord is None:
                self.Print('Bad json file - no version %s' % filename,'E')
                return  ret              
        except Exception as e:
            self.Print(e,'E')
            self.Print('Can not read JSON file %s' % filename,'E')                
            return ret
        
        # update
        data["CamMatrix"] = np.array(mtrx).tolist()
        data["CamDist"]   = np.array(dist.ravel()).tolist()
        # to support all compatability
        #data.setdefault("CamDist", np.array(dist).tolist())
        
        # write back
        try:
            with open(filename, "w") as outfile: 
                #json.dump(data, outfile) 
                json.dump(data, outfile, indent=4) 
            ret = True
        except:
            self.Print('Can not write JSON file %s' % filename,'E')                
            return ret
                   
        self.Print('JSON file is updated with new camera parameters %s' %filename)
        return True

    
    def Print(self, txt='',level='I'):
        
        if level == 'I':
            ptxt = 'I: CFG: %s' % txt
            log.info(ptxt)  
        if level == 'W':
            ptxt = 'W: CFG: %s' % txt
            log.warning(ptxt)  
        if level == 'E':
            ptxt = 'E: CFG: %s' % txt
            log.error(ptxt)  
           
        #print(ptxt)
    
    
    def Finish(self):
        
        self.SaveSession()
        self.Print('Project is closed')
        
             
# --------------------------           
class TestConfigManager(unittest.TestCase):                
    def test_Create(self):
        p = ConfigManager("..")
        self.assertEqual("..", p.path)
                
    def test_ValidateObjectDirectory(self):
        # test params dave a nd load           
        p = ConfigManager("..\parts_ud\Kuku")
        self.assertTrue(p.SetupConfigFile()) 

        # load without use previous file
        p = ConfigManager()
        self.assertTrue(not p.SetupConfigFile()) 
        
        
    def test_ReadWriteConfig(self):
        # test params dave a nd load           
        p               = ConfigManager(r'..\parts_ud\tete')
        #cfg = {}
        cfg             = p.ReadPlainconfig()  
        self.assertNotEqual(cfg,None) 
        cfg['model_name'] = "tete"
        #cfg['scripts_folder'] = r'..\pose6d'
        #cfg['CamMatrix'] = [1,2,3,4,5,6,7,8,9]
        p.WritePlainconfig(cfg)
        cfgR            = p.ReadPlainconfig()  
        self.assertEqual(cfgR,cfg) 
        
               
    def test_UpdateCameraParams(self):
        # test params dave a nd load           
        p           = ConfigManager(r'..\parts_ud\qwqw')
        mtrx        = [1,2,3,4,5,6,7,8,9]
        dist        = [0.1,-0.1,1]
        resol       = (128,64)
        p.UpdateCameraParams(mtrx, dist, resol)  
        cfgR        = p.ReadPlainconfig()  
        self.assertEqual(cfgR['CamMatrix'],mtrx) 
        

        
    def test_UpdateFileList(self):
        # check 4 video files in this folder           
        p           = ConfigManager(r'..\parts_ud\Singapore')
        fileList    = p.GetVideoFileList()  
        p.UpdateVideoFileList(fileList)
        cfgR        = p.ReadPlainconfig()  
        self.assertEqual(len(cfgR['json_folders_list']),4) 
        
    def test_UpdateObjectDimensions(self):
        p           = ConfigManager(r'..\parts_ud\Singapore')
        p.UpdateObjectDimensions()
 
            
# -------------------------- 
if __name__ == '__main__':
    #print(__doc__)
    
#    unittest.main()
    
    # single test
    singletest = unittest.TestSuite()
    #singletest.addTest(TestConfigManager("test_CenterPointHollowObject"))
    #singletest.addTest(TestConfigManager("test_BasicTransforms"))
    #singletest.addTest(TestConfigManager("test_ZeroRxRyTransforms"))    
    #singletest.addTest(TestConfigManager("test_RoboDkTransforms")) # ok
    singletest.addTest(TestConfigManager("test_Create"))
    
    unittest.TextTestRunner().run(singletest)
    
    #p =  ConfigManager(r'..\parts_ud\Singapore')
    # Tests
    #p.OpenProjectDirectory()
    #p.ListDirectoryOfObject()
    #p.SelectObjectDirectory()
    #p.ValidateObjectDirectory()
    #p.CreateObjectDirectory()
    #p.SelectVideoFileList()
    # 
    #p.TestSaveLoad()
    #p.Test()
    