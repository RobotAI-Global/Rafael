# -*- coding: utf-8 -*-
"""
Main GUI for Rafael App.
 
PC : 
enviroment: 
    D:/RobotAI/Design/env/pose6d/python.exe
    
LIRY:
    (C:\robotai\SW\PyEnv\RobotMngr
    
installation:
    conda install -c conda-forge matplotlib
    pip install opencv-python
    conda install -c anaconda scipy


-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0101    11.06.24 UD     Created
-----------------------------

"""

#%% Imports

#import os
import sys
#sys.path.insert(1, r'D:\RobotAI\Customers\EGM\Python\egm\pose6d\gui')
sys.path.insert(1, r'.\\robot')
sys.path.insert(1, r'.\\gui')

__version__ = '0101'


import sys
if hasattr(sys, 'frozen') and hasattr(sys, '_MEIPASS'):
    pass
    #import MonitorGUI

#from gui.MonitorGUI import MainGUI
from control.MainProgram import MainProgram
#from gui.ConfigManager import ConfigManager
#from gui.DisplayManager  import DisplayManager
#from robot.RobotAPI import RobotAPI as RobotManager
#
#
###% Constructions
#"""Constructor method"""
#cfg    = ConfigManager()
##prj    = ProjectManager(config = cfg) # load session
##cam    = CameraManager(config = cfg)
##obj    = ObjectManager(config = cfg)
#rob    = RobotManager(config = cfg)
##lbl    = LabelManager(config = cfg)
##ste    = StereoManager(config = cfg)
#dsp    = DisplayManager(config = cfg)

module_dict = {} #'cfg':cfg, 'rob':rob, 'dsp': dsp}



# --------------------------
if __name__ == '__main__':
    #main(__version__)
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--version", "-V", action="store_true", help="show version"
    )
    parser.add_argument(
         #"--gui", action="store_true", help="show GUI"
         "--gui", action="store_false", help="disable GUI and run server"
    )
    parser.add_argument(
         "--start", action="store_true", help="starts to run automatically"
    )


    args = parser.parse_args()
    if args.version:
        print("{0} {1}".format('MainApp', __version__))

#    if args.start:
#        #f start to run automatically
#        MainGUI(__version__, module_dict)
#    
#
#    if args.gui:
#        #from gui import RobotAIGUI
#        MainGUI(__version__, module_dict)
#    else:
    MainProgram(__version__, module_dict)
# #        from gui import RobotAIAPI
#             RobotAIAPI.AppServer(__version__, module_dict)
    
    #sys.exit(0)
