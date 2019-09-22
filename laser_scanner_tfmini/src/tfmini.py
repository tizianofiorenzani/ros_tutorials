<<<<<<< HEAD
# -*- coding: utf-8 -*
import serial
import time

class TfMini():

    def __init__(self, serial_port="/dev/ttyAMA0"):

        self._ser = serial.Serial(serial_port, 115200)
        if self._ser.is_open == False:
            self._ser.open()

        self._distance = 0
        self.distance_min   = 10
        self.distance_max   = 1200

    def get_data(self):
        
        time0 = time.time()
        while True:
            count = self._ser.in_waiting
            distance = -1
            if time.time() > time0 + 1: break
            if count > 8:
                recv = self._ser.read(9)
                self._ser.reset_input_buffer()
                if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                    low = int(recv[2].encode('hex'), 16)
                    high = int(recv[3].encode('hex'), 16)
                    distance = low + high * 256
                    break
                    
        self._distance = distance
        return(distance)
        
    @property
    def distance(self):
        return(self._distance)
        
    def print_data_thread(self):
        while True:
            print(self.get_data())
            time.sleep(0.5)
            
    def close(self):
        if self._ != None:
            self._ser.close()
        


if __name__ == '__main__':
    tfmini = TfMini()
    # print tfmini.get_data()
    tfmini.print_data_thread()
=======
# -*- coding: utf-8 -*
#-- Derived from https://github.com/TFmini/TFmini-RaspberryPi

import serial
import time

class TfMini():

    def __init__(self, serial_port="/dev/ttyAMA0"):

        self._ser = serial.Serial(serial_port, 115200)
        if self._ser.is_open == False:
            self._ser.open()

        self._distance = 0
        self.distance_min   = 10
        self.distance_max   = 1200

    def get_data(self):
        
        time0 = time.time()
        while True:
            count = self._ser.in_waiting
            distance = -1
            if time.time() > time0 + 1: break
            if count > 8:
                recv = self._ser.read(9)
                self._ser.reset_input_buffer()
                if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                    low = int(recv[2].encode('hex'), 16)
                    high = int(recv[3].encode('hex'), 16)
                    distance = low + high * 256
                    break
                    
        self._distance = distance
        return(distance)
        
    @property
    def distance(self):
        return(self._distance)
        
    def print_data_thread(self):
        while True:
            print(self.get_data())
            time.sleep(0.5)
            
    def close(self):
        if self._ != None:
            self._ser.close()
        


if __name__ == '__main__':
    tfmini = TfMini()
    # print tfmini.get_data()
    tfmini.print_data_thread()
>>>>>>> 339226fe7f5a20562103b4bc3e9d6b0802dcd869
