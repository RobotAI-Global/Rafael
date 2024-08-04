# -*- coding: utf-8 -*-
"""
State Machine class
Implements the logic 

@author: zion
"""

# Logger
import logging as log
#import sys

log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")


#%% 
# StateMachine/State.py
# A State has an operation, and can be moved
# into the next State given an Input:
from enum import Enum
class STATE(Enum):
    INIT = 1    
    FINISH = 100
    ERROR  = 401
    
    
        
#%% State Machine of the process        
    
class StateMachine:
    def __init__(self, parent = None):        
        self.parent = parent
        
        self.currentState = STATE.INIT
        #self.currentState.run()
        self.tprint('started')
        
    def tprint(self, txt = ''):
        #print(txt)
        #log.info(txt)
        log.info(f'{self.currentState} : {txt}')
               
            
            
#%% Testing - unittest
class TestStateMachine:
    def __init__(self):
        self.sm = StateMachine()
        
    def test_current_state(self):
        self.sm.tprint('1')
        assert self.sm.currentState == STATE.INIT
        
    def test_all(self):
        self.test_current_state()
        

#%%
            
if __name__ == '__main__':
    from StateMachine import TestStateMachine as TestSM
    tsm = TestSM()
    tsm.test_all()
