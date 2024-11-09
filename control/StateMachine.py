# -*- coding: utf-8 -*-
"""
State Machine class
Implements the logic 

@author: zion
"""

# # Logger
# import logging as log
# #import sys

# log.basicConfig(level=log.DEBUG, format='[%(asctime)s.%(msecs)03d] {%(filename)s:%(lineno)d} %(levelname)s - %(message)s',  datefmt="%M:%S")


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
        self.Print('started')
        
    def Init(self):
        "initial state"
        self.currentState = STATE.INIT
        self.Print('Init')
        
    def StateSpecialMessage(self, msg_in, curr_state):
        "special messages"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state
    
    def StateInit(self, msg_in, curr_state):
        "do onthing"
        msg_out     = msg_in
        next_state  = curr_state
        return msg_out, next_state    
        
    def Transition(self, msg_in):
        "transition to a different state"
        curr_state = self.currentState
        next_state = self.currentState

        
        # deal with special messages
        msg_out, curr_state = self.StateSpecialMessage(msg_in, curr_state)
        
        # deal with messages per state
        if curr_state == STATE.INIT:
            msg_out, next_state = self.StateInit(msg_in, curr_state)
        elif curr_state == STATE.FINISH:
            msg_out, next_state = self.StateFinish(msg_in, curr_state)
        else:
            self.Print('Not supprted state')
            pass
        
        self.Print('Transition to %s' %str(next_state))   
        self.currentState = next_state
        return msg_out
        
        
    def Print(self, txt = ''):
        print('I: STM: %s' %str(txt))
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
