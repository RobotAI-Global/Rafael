"""

API to the robot


"""

try: # running from top
    from robot.Robot  import Robot
    from robot.Packet import Packet
except:
    print('running locally')
    from Robot  import Robot
    from Packet import Packet

from queue import Queue
from threading import Thread

import time
import numpy as np
# Logger
import logging as log
import sys
log.basicConfig(stream=sys.stdout, level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")

SCAN_POINTS  = [[-500,-200, 400, 94, 0, -82],[-500,0,400, 94, 0, -82],[-500,0,600, 94, 0, -82],[-500,-220,600, 94, 0, -82],[-665.01 ,-251.23 , 648.96 , 94, 0, -82]]


WORK_POINTS  = [[-0.520,-0.200, 0.400, -1.64,0, 1.7],   [-0.750,-0.200, 0.400, -1.64,0, 1.7],
                [-0.500, 0,     0.400, -1.64,0, 1.7],   [-0.750, 0,     0.400, -1.64,0, 1.7],
                [-0.500, 0,     0.600, -1.64,0, 1.7],   [-0.650, 0,     0.600, -1.64,0, 1.7],
                [-0.500,-0.220, 0.600, -1.64,0, 1.7],   [-0.750,-0.220, 0.600, -1.64,0, 1.7]]

#%% Main function
class RobotAPI:
    def __init__(self, parent=None):
        #super().__init__()
        self.parent         = parent
        self.r              = Robot()

        # Init  Neura robot class
        #self.r = Robot()
        
        self.stopRobotTask = False
        
        # Init queue with data - to robot thraed transfer by server thread (recived data)
        self.queueToRobot   = Queue()
        # Init queue with data - to server thread transfer by robot thraed (recived data)
        self.queueFromRobot = Queue()
        
        # Init local var msgPacketInternal with Packet to send back new data to server
        # It will be send to the client with server thread
        self.msgPacketInternal = Packet()
        
        # Init var RobotIsBusy 
        self.RobotIsBusy       = False
        
        self.ts                 = None
  
        
    def Init(self):
        "bring the process to the initial state"
        self.RobotIsBusy       = False
        self.stopRobotTask     = False
        while not self.queueToRobot.empty():
            self.queueToRobot.get()
        while not self.queueFromRobot.empty():
            self.queueToRobot.get()
        print('Clearing queue and init')      


# ------ Robot Tasks ------ 
    # When last_cmd_status = 0 (Task Done Successfully)  
    # When last_cmd_status = 1 (Task Fail)  
    # When last_cmd_status = 2 (Task in Process)
    def HomeTask(self):        
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Robot is moving to HOME position in process....') 
        time.sleep(20) 
        print('Robot HOME done - status 0')
        self.msgPacketInternal.last_cmd_status = 0
        self.RobotIsBusy       = False
        
    def StopTask(self):        
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Robot STOP in process....') 
        time.sleep(20) 
        print('Robot STOP done - status 1')
        self.msgPacketInternal.last_cmd_status = 0   
        self.RobotIsBusy       = False
        
    def LoadUUTToTableTask(self):
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Loading UUT to index table in process....') 
        time.sleep(20) 
        print('Loading UUT to index table done - status 2')
        self.msgPacketInternal.last_cmd_status = 0   
        self.RobotIsBusy       = False
        
    def UnloadUUTFromTableTask(self):
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Unloading UUT from index table in process....') 
        time.sleep(20) 
        print('Unloading UUT from index table done - status 3')
        self.msgPacketInternal.last_cmd_status = 0   
        self.RobotIsBusy       = False
        
    def LoadUUTToTestStandTask(self):
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Loading UUT to test stand in process....') 
        time.sleep(20) 
        print('Loading UUT to test stand done - status 4')
        self.msgPacketInternal.last_cmd_status = 0   
        self.RobotIsBusy       = False

    def UnloadUUTFormTestStandTask(self):
        self.RobotIsBusy       = True
        self.msgPacketInternal.last_cmd_status = 2
        print('Unloading UUT from test stand in process....') 
        time.sleep(20) 
        print('unloading UUT from test stand done - status 5')
        self.msgPacketInternal.last_cmd_status = 0   
        self.RobotIsBusy       = False        

    # ------ Commands Threads ------        
    def Home(self):
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('Go Home Thread start') 
        # Check if thread is already Init
        if self.ts is not None:
            # Check if thread is already ruuning
            if self.t1.isAlive():
                print('Thread is already running')
                return
        
        self.t1 = Thread(target = self.HomeTask)
        self.t1.start() 
                
        # print('Naura Robot type')
        #print(self.r.robot_name)
        
    def Stop(self):        
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('Robot STOP Thread start')
        # Check if thread is already Init
        if self.t2 is not None:
            # Check if thread is already ruuning
            if self.t2.isAlive():
                print('Thread is already running')
                return
        t2 = Thread(target = self.StopTask)
        t2.start()
        
        # print('Robot stop working...') 
        
    def LoadUUTToTable(self):
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('load UUT to index table Thread start')
        # Check if thread is already Init
        if self.t3 is not None:
            # Check if thread is already ruuning
            if self.t3.isAlive():
                print('Thread is already running')
                return
        t3 = Thread(target = self.LoadUUTToTableTask)
        t3.start() 
        
    def UnloadUUTFromTable(self):
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('Unload UUT from index table Thread start')
        # Check if thread is already Init
        if self.t4 is not None:
            # Check if thread is already ruuning
            if self.t4.isAlive():
                print('Thread is already running')
                return
        t4 = Thread(target = self.UnloadUUTFromTableTask)
        t4.start()
        
    def LoadUUTToTestStand(self):
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('Load UUT to test stand Thread start')
        # Check if thread is already Init
        if self.t5 is not None:
            # Check if thread is already ruuning
            if self.t5.isAlive():
                print('Thread is already running')
                return
        t5 = Thread(target = self.LoadUUTToTestStandTask)
        t5.start()
        
    def UnloadUUTFromTestStand(self):
        if self.RobotIsBusy:
            print('Robot is busy....')
            return 
        
        print('Unload UUT from test stand Thread start')
        # Check if thread is already Init
        if self.t6 is not None:
            # Check if thread is already ruuning
            if self.t6.isAlive():
                print('Thread is already running')
                return
        t6 = Thread(target = self.UnloadUUTFormTestStandTask)
        t6.start()        
        
        # ------ Connecting to Robot ------        
    def Connect(self):
        print('Connecting to robot....')        

    # ------ Executing Commands ------        
    def ExecutePacket(self):
        
        if self.queueToRobot.empty():
            return
        
        msgPacket  = self.queueToRobot.get()
        
        if not isinstance(msgPacket,Packet):
            print('Wrong object')         
    
        if msgPacket.command == 0:
            print('0 - DEFULT command')           
            self.Home()
            
        elif msgPacket.command == 1:
            print('1 - STOP command')
            self.Stop()
            
        elif msgPacket.command == 2:
            print('2 - Load UUT to index table command')
            self.LoadUUTToTable()
            # self.r.move_joint(target_joint=[0,90,0,90,0,0],speed=100,accelearation=70)
            # self.r.move_joint(target_joint=[0,0,90,0,0,0],speed=100,accelearation=70)
            
        elif msgPacket.command == 3:
            print('3 - Unload UUT from index table command')
            self.UnloadUUTFromTable()
            
        elif msgPacket.command == 4:
            print('4 - Load UUT to test stand command')
            self.LoadUUTToTestStand()
            
        elif msgPacket.command == 5:
            print('5 - Unload UUT from test stand command')
            self.UnloadUUTFromTestStand()
            
        elif msgPacket.command == 6:
            print('6 - Get staus command')
            msgPacket.msgSize = 60
            msgPacket.cmdCode = 53
            
            msgPacket.last_cmd_status = self.msgPacketInternal.last_cmd_status
            msgPacket.general_status[0] = 1
            msgPacket.general_status[1] = 2
            msgPacket.general_status[2] = 3
            msgPacket.general_status[3] = 4
            msgPacket.general_status[4] = 5
            msgPacket.general_status[5] = 6
            msgPacket.general_status[6] = 7
            msgPacket.general_status[7] = 8
            msgPacket.general_status[8] = 9
            msgPacket.general_status[9] = 10
            
            # Preparing data to be send to client 
            # Sending packet only when commands are 6 or 7 
            # Other commands just execute robot movments and logic
            self.queueFromRobot.put(msgPacket)            
            
        elif msgPacket.command == 7:
            print('7 - Get bit results command')
            msgPacket.msgSize = 64
            msgPacket.cmdCode = 52                
            
            # msgPacket.bit_status     = self.msgPacketInternal.bit_status
            msgPacket.bit_status     = 1
            msgPacket.seconds = 45
            msgPacket.error_codes[0] = 1
            msgPacket.error_codes[1] = 2
            msgPacket.error_codes[2] = 3
            msgPacket.error_codes[3] = 4
            msgPacket.error_codes[4] = self.msgPacketInternal.error_codes[4]
            msgPacket.error_codes[5] = 6
            msgPacket.error_codes[6] = 7
            msgPacket.error_codes[7] = 8
            msgPacket.error_codes[8] = 9
            msgPacket.error_codes[9] = 10 
            
            # Preparing data to be send to client
            # Sending packet only when commands are 6 or 7 
            # Other commands just execute robot movments and logic
            self.queueFromRobot.put(msgPacket)
            
        elif msgPacket.command == 8:
            print('8 - Connections counter zeroise command')   
            
        else:
            print('ERROR - wrong command %s' %str(msgPacket.command))


    def RobotThread(self):  
        "main thread"        
        self.Connect()    
        print('Starting robot thread')
        self.Init()
        while not self.stopRobotTask:
            try:                    
                self.ExecutePacket()
                    
            except Exception as e:
                print(e)  
                self.stopRobotTask = True
                
    def RunThread(self):
        "running in the thread"
        self.ts = Thread(target = self.RobotThread)
        self.ts.start()
        #self.ts.join()    
        return True    

    def CheckThreadIsAlive(self):
        return self.ts.isAlive()

    def StopThread(self):
        "stop running thread"
        self.stopRobotTask        = True
         
        
    def CloseConnection(self):
        # close the connection
        #self.conn.close() 
        self.Print('TBD')                                      
                
        
    def Connected(self):
        # check if the robot exists
        try:
            self.Print('Robot Name: ', self.r.robot_name)
            ret = True
        except:
            self.Print('Can not locate the robot')
            ret = False
        return ret
        
    def RobotData(self): 
        "show some robot info"
        self.r.robot_info() 
#        self.Print('Robot Name: ', self.r.robot_name)
#        self.Print('Number Of Axis: ', self.r.dof)
#        self.Print('robot IP Address: ', self.r.kURL)
#        self.Print('Home Position: ', self.r.get_point("Home"))
#        self.Print('Current Joint Angles: ', self.r.robot_status("jointAngles"))
#        self.Print('Current Position: ', self.r.robot_status("cartesianPosition"))
        
    def RobotPower(self, value = 'on'):
        # r.power('on') / r.power('off')
        if value in ['on','off']:
            self.r.power(str(value))
        else:
            self.Print(f'input must be on or off - given {value}')
            
    def GetRobotStatus(self):
        sts = self.r.robot_status()
        return sts
        
    def RobotStop(self, value):
        # stop the robot
        self.r.stop()        
        
    def MoveJoint(self, JointAngles, j_Speed, j_Acc):
        #r.move_joint(p)
        
        # target_joint = [0.3,0.2,0.3,0,0,0]
        # r.move_joint(target_joint,speed=100,accelearation=70)
        
        # p = r.get_point("Home")
        # target_joint2 = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
        # r.move_joint(target_joint2,speed=100,accelearation=70)
        
        self.r.move_joint(JointAngles, speed = j_Speed, accelearation = j_Acc)
        self.Print("move_joint")
        
    def MoveLinear(self, Position, l_Speed, l_Acc):
        #r.move_linear(p)
        
        # target_linear = [25,30,35,0,0,0]
        # r.move_linear(target_linear,speed=0.8,accelearation=1)
        
        # p = r.get_point("Home")
        # target_Position = [p['originX'], p['originY'], p['originZ'],
        #                   p['originA'], p['originB'], p['originC']]
        # r.move_linear(target_Position,speed=0.5,accelearation=0.5)
        
        self.r.move_linear(Position, speed = l_Speed, accelearation = l_Acc)
        self.Print("move_linear")
        
    def GetInputStatus(self, InputName):
        #print(r.io("get",io_name="DO_1"))
        self.InputValue = self.r.io("get",io_name = str(InputName))
        
        if self.InputValue == 0:
            val = False
        else:
            val = True
            
        return val
    
    def SetOutput(self, OutputName, Action):
        # r.io("set",io_name="DO_1",target_value=True)
        
        if Action == 'on':
            self.r.io("set",io_name = str(OutputName),target_value = True)
        else:
            self.r.io("set",io_name = str(OutputName),target_value = False)
            
    def SetGripper(self, Action):
        # r.gripper("close")
        self.r.gripper(str(Action))
        
    def GetPosition(self, g_Type, g_PosName):
        #p    = r.get_point("Home")
        #p['a1'] = -p['a1']
        
        p = self.r.get_point(str(g_PosName))
        
        if g_Type == 'Position':            
            target = [p['originX'], p['originY'], p['originZ'],
                              p['originA'], p['originB'], p['originC']]
        elif g_Type == 'Joint':
            target = [p['a1'], p['a2'], p['a3'], p['a4'], p['a5'], p['a6']]
            
        return target  
    
    def GetCurrent(self, g_Type):
        if g_Type == 'Position':
            current = self.r.robot_status("cartesianPosition")
            
        elif g_Type == 'Joint':
            current = self.r.robot_status("jointAngles")
          
        return current
    
    def DegreeToRadians(self, deg):        

        radians = np.deg2rad(deg)
    
        print(f"{deg} degrees = {self.radians} radians")
        
        return radians
    
    def RadiansToDegree(self, rad):
        
        degrees = np.rad2deg(rad)

        print(f"{rad} radians = {self.degrees} degrees")
        
        return degrees
    
    ## -----------------------------    
    # -- Robot Control ---
    ## -----------------------------    
        
    def RobotConnect(self):
        # start
        self.tprint('Starting Robot connection ...')
        
        # maybe already running
        if self.rbm is None:
            # runs Robot server and Multi Object Detection
            self.tprint('Bad Robot Init  ... Abort') 
            return
            #self.rbm    = RobotManager(ip = self.ip_robot, port = self.port_robot, block_com=False, config=self.cfg) #RobotServerThread(host = self.ip, port = self.port,  debug=self.debugOn, config = self.cfg)
            #self.rbm.instanceSocket()
            #self.rbm.start()
        elif self.rbm.Connected():   
            self.tprint('Robot is alive')
        else:
            self.rbm.RobotData()
            self.tprint('Robot is restarted')
            
        #self.rbm.setPowerOn()
        #self.rbm.setReleaseBrake()

    def RobotStop(self):
        # start
        self.tprint('Stop Robot  ...')
        
        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return
        
        self.rbm.Stop()

    def RobotCommState(self):
        # comm problems?
        
        comm_state = self.rbm.GetRobotStatus()
        self.tprint("Robot status: %s" %str(comm_state))

    def RobotStatus(self):
        # start
        self.tprint('Getting Robot status ...')

        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return
        
        stat = self.rbm.GetRobotStatus()
        self.tprint(str(stat))


    def RobotDisConnect(self):
        # start
        self.tprint('Disconnect from Robot ...')
        
        # maybe already running
        if self.rbm is None:
            # runs 
            pass
        elif self.rbm.Connected():  
            self.rbm.Stop()
            self.rbm.RobotPower('off')            
            #self.rbm.stop()
        else:
            self.rbm.RobotPower('off')

        self.tprint('Robot is stopped')  
        
    def RobotSetRobotSpeed(self):
        # set speed 1-100
        speedVal = np.maximum(10,np.minimum(200,self.sliderCount))        
        self.robot_speed = speedVal
        
        self.tprint('Robot speed set : %s' %str(speedVal)) 
        
    def RobotGetGripperPose(self):
        # read pose
        self.tprint('Robot read pose ... ') 
        
        # maybe already running
        if self.rbm is None or self.rbm.Connected() is False:
            self.tprint('Connect to the robot first ...')
            return [0]*6
        
        res = self.rbm.GetPosition() #getTool_xyzrxryrz()
        robotPose = res[1]
        
        robotPose[0] = robotPose[0] * 1000
        robotPose[1] = robotPose[1] * 1000
        robotPose[2] = robotPose[2] * 1000
        self.tprint('Robot pose is (rad): %s' %str(robotPose))
        robotPose[3] = np.rad2deg(robotPose[3])
        robotPose[4] = np.rad2deg(robotPose[4])
        robotPose[5] = np.rad2deg(robotPose[5])
        self.tprint('Robot pose is : %s' %str(robotPose))

        self.robot_pose = robotPose
        return robotPose
    
    def RobotSetGripperPose(self):
        # set pose
        self.tprint('Robot set pose GUI ... ') 
        
       
        
    def RobotMarkWorkPosition(self):
        # read pose
        self.tprint('Robot shows work area ... ') 

        
        val = 0
        for robotPose in WORK_POINTS:
            print(f"??????????? ?: {robotPose}")
            self.rbm.setmovel(robotPose)
            time.sleep(0.5)
            self.rbm.setDO(1,val)
            print('Robot gripper command %d ... ' %val) 
            val = 1 - val

    def RobotGoHome(self):
        # read pose
        self.tprint('Robot home pose ... ') 
               
        # res = self.rbm.getmovel() #getTool_xyzrxryrz()
        # isOk, robotPose, msg = res
        # if not isOk:
        #     return

        robotPose = self.home_pose #[-0.689, -0.121, 1.035, -2.0, 0.0, 2.0]
        self.robotAbsoluteMovePose(robotPose)

    def RobotSetHomePose(self):
        # setting home pose
        self.tprint('Robot set current pose to be home pose ... ') 
        
        robotPose = self.robotGetGripperPose()        
        self.home_pose[0] = robotPose[0]
        self.home_pose[1] = robotPose[1]
        self.home_pose[2] = robotPose[2]

    def RobotGripperOnOff(self,val = 1):
        # set gripper
        if val < 0.5:
            self.rbm.SetGripper('close')
        else:
            self.rbm.SetGripper('open')
            
        self.tprint('Robot gripper command %d.' %(val)) 
        
    def RobotDiffMovePose(self, dPose = np.zeros((1,6))):
        # read pose
        # maybe already running
        if self.rbm is None or self.rbm.connected() is False:
            self.tprint('Connect to the robot first ...')
            return

        self.tprint('Robot read pose and move ... ')         
        
        res = self.rbm.getmovel() #getTool_xyzrxryrz()
        isOk, robotPose, msg = res
        self.tprint('Robot pose is : %s' %str(np.round(robotPose,2)))
        if not isOk:
            self.tprint('Can not read position')
            return
        
        # dPose[0] = dPose[0] / 1000
        # dPose[1] = dPose[1] / 1000
        # dPose[2] = dPose[2] / 1000
        # dPose[3] = np.deg2rad(dPose[3])
        # dPose[4] = np.deg2rad(dPose[4])
        # dPose[5] = np.deg2rad(dPose[5])

        robotPose[0] = robotPose[0] + dPose[0]
        robotPose[1] = robotPose[1] + dPose[1]
        robotPose[2] = robotPose[2] + dPose[2]
        robotPose[3] = robotPose[3] + dPose[3]
        robotPose[4] = robotPose[4] + dPose[4]
        robotPose[5] = robotPose[5] + dPose[5]
        
        self.tprint('Going to Robot pose %s ' %str(robotPose))
        self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2= str(self.robot_speed), num3= str(self.robot_speed))
        
    def RobotAbsoluteMovePose(self, dPose = [0]*6):
        # move to pose

        # maybe already running
        # if self.rbm is None or self.rbm.connected() is False:
        #     self.tprint('Connect to the robot first ...')
        #     return 

        #self.tprint('Robot read pose and move ... ') 
        self.tprint('Going to Robot pose [mm,deg] %s ' %str(np.round(dPose,2)))

        # res = self.rbm.getmovel() #getTool_xyzrxryrz()
        # isOk, robotPose, msg = res
        # #self.tprint('Robot pose is : %s' %str(robotPose))
        # if not isOk:
        #     self.tprint('Can not read position')
        #     return
        robotPose    = dPose.copy() #np.array(dPose).copy()
        
        robotPose[0] = dPose[0] / 1000
        robotPose[1] = dPose[1] / 1000
        robotPose[2] = dPose[2] / 1000
        robotPose[3] = np.deg2rad(dPose[3])
        robotPose[4] = np.deg2rad(dPose[4])
        robotPose[5] = np.deg2rad(dPose[5])

        
        #self.tprint('Going to Robot pose [m,rad]%s ' %str(robotPose))
        #self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2='100', num3='100')
        #isOk, robotPose, msg = self.rbm.setmovel(robotPose, num1 = str(self.robot_speed), num2= str(self.robot_speed), num3= str(self.robot_speed))
        self.rbm.setmovel(robotPose)
        return 
    
    
    ## ------------------------------------  
    # -- Task --      
    ## ------------------------------------ 
        
    def RobotMultiPointMotion(self):
        # check multi point motion as deefined by China
        self.tprint('Starting point motion ...')

        val = 1
        pointNum  = len(SCAN_POINTS)
        for k in range(pointNum):

            #self.robotCommState()
            robotPose = SCAN_POINTS[k].copy()

            self.tprint('Moving....')
            self.robotAbsoluteMovePose(robotPose)
            time.sleep(0.1)

            self.tprint('Gripper on-off')
            self.robotGripperOnOff(val)
            val = 1 - val

            self.tprint('Finishing scan point %d from %d' %(k+1,pointNum))
            

        self.tprint('Robot finished the scan.')


    def RobotDetectAndMoveToPoint(self):
        # start
        #self.tprint('Client sending request to robot. Go Home Pose ...')
        #self.robotAbsoluteMovePose([-356,-654,285,-90,0,140])

        self.tprint('Client sending request to robot ...') 
        robotPose = self.robotGetGripperPose()

        #self.tprint('Client sending request to host ...') 
        objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPose)
        self.tprint('Received Object pose [mm,deg] %s ' %str(np.round(objPose,2)))
        if objQ < 0.7:
            self.tprint('Object is not detected. Not moving.')
            return

        self.tprint('Client sending Command to the Robot Move Linear ...')
        #objPose[0] = objPose[0] + SAFETY_MARGIN_Z # actually X
        self.robotAbsoluteMovePose([objPose[0],objPose[1],objPose[2],robotPose[3],robotPose[4],robotPose[5]])
        # self.robotAbsoluteMovePose([-400,-700,robotPose[2],robotPose[3],robotPose[4],robotPose[5]])

        self.tprint('Robot is at object pose - check.')

    def RobotScanDetectMove(self):
        # scan multiple position detect and move to the points
        self.tprint('Starting scan %s times...' %str(self.sliderCount))

        pointNum  = len(SCAN_POINTS)
        for repeatNum in range(self.sliderCount):
            self.tprint('Repeat %s ...' %str(repeatNum))
            for k in range(pointNum):

                self.tprint('Going to point %d ....' %(k+1))
                self.robotAbsoluteMovePose(SCAN_POINTS[k])
                #time.sleep(15)

                self.tprint('Touch....')
                self.robotDetectAndMoveToPoint()
                #time.sleep(10)

                self.tprint('Going from point %d ....' %(k+1))
                self.robotAbsoluteMovePose(SCAN_POINTS[k])
                #time.sleep(10)
                self.tprint('Finishing scan point %d from %d' %(k+1,pointNum))
            

        self.tprint('Robot finished the scan.')

    def RobotDetectMoveAndRepeat(self):
        # touches the point several times
        SMALL_DIFF_POSE    = [[ 0,  0, 0, 0 ,0, 0],
                              [ 0, -50, 0, 0, 0, 0],
                              [ 50,-50, 0, 0, 0, 0],                              
                              [ 50,  0, 0, 0, 0, 0]]
        pointNum = len(SMALL_DIFF_POSE)
        self.tprint('Repeating %d times ...' %pointNum) 
        robotPose = self.robotGetGripperPose()

        for k in range(pointNum):
            self.tprint('Client sending request to robot ...') 
            robotPoseTemp = robotPose.copy()
            robotPoseDiff = SMALL_DIFF_POSE[k]
            robotPoseTemp[0] += robotPoseDiff[0]
            robotPoseTemp[1] += robotPoseDiff[1]
            self.robotAbsoluteMovePose(robotPoseTemp)
            time.sleep(0.5)

            #self.tprint('Client sending request to host ...') 
            objId, objNum, objPose, objQ = self.hostSendRecv(robotPose=robotPoseTemp)
            self.tprint('Received Object pose [mm,deg] %s ' %str(np.round(objPose,2)))
            if objQ < 0.7:
                self.tprint('Object is not detected. Not moving.')
                break

            self.tprint('Client sending Command to the Robot Move Linear ...')
            #objPose[0] = objPose[0] + SAFETY_MARGIN_Z # actually X
            self.robotAbsoluteMovePose([objPose[0],objPose[1],objPose[2],robotPose[3],robotPose[4],robotPose[5]])
            # self.robotAbsoluteMovePose([-400,-700,robotPose[2],robotPose[3],robotPose[4],robotPose[5]])

            self.tprint('Robot is at object pose - check.') 

        self.robotAbsoluteMovePose(robotPose)       
    
    
    def Print(self, txt = ''):
        #print(txt)
        #log.info(txt)
        log.info(txt)    

    # ------ For testing only ------
#    def TestRobot(self):
#        print('testing robot')
#         
#        print(self.r.robot_name)
#        print(self.r.dof)
#        print(self.r.kURL)
#        print(self.r.move_joint(target_joint=[0,0,0,0,0,0],speed=100,accelearation=70))
#        # print(r.get_point("Home"))
#        # print(r.robot_status("jointAngles"))
#        # print(r.io("get",io_name="DO_1"))
#
#        #string_value = tcp_client.receive_Data()
#        io_value = self.r.io("get",io_name="DO_1")
#
#        if io_value == 0:
#            self.r.move_joint()
#            self.r.gripper("close")
#            self.r.move_linear()
#            self.r.io("set",io_name="DO_1",target_value=True)           
       
#%% Tests           
class TestRobotAPI: #unittest.TestCase
    
    def __init__(self):
        self.rapi = RobotAPI()
        
                     
    def TestRunThread(self):
        "testing thread start"
        #self.rapi.Init() 
        self.rapi.RunThread() 
        time.sleep(10)
        self.rapi.StopThread()
        isOK = self.rapi.CheckThreadIsAlive()
        print('Check', isOK)
        
            
            # self.MoveLinear(CurrentPosition, 0.5, 0.5)
            
#%%
            
if __name__ == '__main__':
    #from RobotAPI import TestRobotAPI
    tapi = TestRobotAPI()
    tapi.TestRunThread()
