import socket
import ast
from types import MethodType
import logging as log

def generate_function(function_name,address):
    def wrapped_function(self,*args,**kwargs):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(address)
        data = {'function':function_name,'args':args,'kwargs':kwargs}
        sock.sendall(str(data).encode('utf-8'))
        new_data = sock.recv(1024)
        sock.close()
        return ast.literal_eval(new_data.decode('utf-8'))
    wrapped_function.__name__ = function_name+'_method'
    return wrapped_function


class Robot:
    def __init__(self):
        self.__server_address = ('192.168.2.13',65432)
        log.info(str(self.__server_address))
        
    def connect(self):
        self.__functions = ['move_joint','move_linear', 'move_circular', 'move_composite', 'record_path', 'power',\
            'zero_g', 'io', 'set_tool', 'gripper', 'wait', 'override', 'pause', 'unpause', 'stop', 'ik_fk', \
            'robot_status', 'program_status','get_point','get_warnings','get_errors','motion_status','initialize_attributes']
        for function in self.__functions:
            setattr(self, function, MethodType(generate_function(function,self.__server_address), self))
        
        for key, value in self.initialize_attributes().items():
            setattr(self,key,value)
