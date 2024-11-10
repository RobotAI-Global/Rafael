# RobotManager
Manager of the robot for Rafael System

# creating virtual env
c:\python -m venv C:\robotai\SW\PyEnv\RobotMngr
where python

# activating
C:\robotai\SW\PyEnv\RobotMngr\Scripts\activate.bat

# updating pip
py -m pip install --upgrade pip

# insralling packages
py -m pip install matplotlib
py -m pip install opencv-python
py -m pip install scipy

# compile
pip install pyinstaller
pyinstaller --onefile MainApp.py 
