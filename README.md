# RobotManager
Manager of the robot and perepherial equipment in the integrated system

# Installation Windows

1 . creating virtual env
C:\python -m venv C:\robotai\SW\PyEnv\RobotMngr

2. activating
C:\robotai\SW\PyEnv\RobotMngr\Scripts\activate.bat

3. updating pip
py -m pip install --upgrade pip

4. installing packages
py -m pip install matplotlib
py -m pip install scipy

5. network
configure your computer to Ethernet IP : 192.168.2.30

# Compile
pip install pyinstaller
pyinstaller --onefile MainApp.py 
