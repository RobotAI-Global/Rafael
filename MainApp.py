# -*- coding: utf-8 -*-
"""
Main GUI for Rafael App.
 
Zion - PC environment:  
    Spyder :     D:/RobotAI/Design/env/pose6d/python.exe
    VSCode :     RVC
    
LIRY:
    C:\robotai\SW\PyEnv\RobotMngr
    
installation:
    conda install -c conda-forge matplotlib
    
"""

__version__ = '0303'

"""
-----------------------------
 Ver    Date     Who    Descr
-----------------------------
0303    10.11.24 UD     gripper on - off
0302    09.11.24 UD     logger improve
0201    18.10.24 UD     Merging SaarSM
0101    11.06.24 UD     Created
-----------------------------

"""

#%% Imports

import sys
sys.path.insert(1, r'.\\robot')
sys.path.insert(1, r'.\\gui')
sys.path.insert(1, r'.\\control')
sys.path.insert(1, r'.\\disc')



import sys
if hasattr(sys, 'frozen') and hasattr(sys, '_MEIPASS'):
    pass
    #import MonitorGUI


    #from control.MainProgram import MainProgram
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

from gui.MonitorGUI import MainGUI

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
    if args.gui:
        #from gui import RobotAIGUI
        MainGUI(__version__, module_dict)

    
    #sys.exit(0)
