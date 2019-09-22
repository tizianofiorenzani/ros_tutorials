<<<<<<< HEAD
"""
Servoblaster from https://github.com/richardghirst/PiBits/tree/master/ServoBlaster

 Servo number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3             21/27           P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22
          
Change the /etc/init.d/servoblaster file to OPTS="--idle-timeout=2000 --pcm --p1pins=<list>"

You can set the actual pin you want to dedicate to be servo with --p1pins option

"""

class ServoBlaster():
    _servo_dict = { 4: 0, 
                    17: 1,
                    18: 2,
                    21: 3,
                    22: 4,
                    23: 5,
                    24: 6,
                    25: 7}
                    
    def __init__(self, gpio_port):
        
        self._gpio_port     = gpio_port
        self._servo_number  = self._servo_dict[gpio_port]

    def update(self, duty_us):
        servoStr ="%u=%u\n" % (self._servo_number, long(duty_us*0.1))
        with open("/dev/servoblaster", "wb") as f:
            f.write(servoStr) 
            
class ServoAngle():
    def __init__(self, gpio_port, angle_min, angle_max, min_duty, max_duty):
        self._gpio_port = gpio_port
        
        self._angle_to_duty = (max_duty - min_duty)/(angle_max - angle_min)
        self._duty_min        = min_duty
        self._angle_min        = angle_min
        self._angle_max        = angle_max
        
        self._angle = 0
        
        self._servo            = ServoBlaster(gpio_port)
        
    def angle_to_duty(self, angle):
        duty = (angle - self._angle_min)*self._angle_to_duty + self._duty_min
        return (duty)
        
    def update(self, angle):
        self._servo.update(self.angle_to_duty(angle))
        self._angle = angle
        
    def set_to_min(self):
        self.update(self._angle_min)
        
    def set_to_max(self):
        self.update(self._angle_max)    
        
    def set_to_middle(self):
        self.update((self._angle_min + self._angle_max)*0.5)   

    @property
    def angle(self):
        return(self._angle)
        
    @property
    def angle_min(self):
        return(self._angle_min)
        
    @property
    def angle_max(self):
        return(self._angle_max)        
    
        
if __name__ == "__main__":
    import time
    #-- ROS Convention: counter clockwise is positive (left positive, right negative)
    servo_angle = ServoAngle(23, -85, 85, 2250, 750)
    
    angle = -85
    sleep = 0.01
    step = 2
    
    servo_angle.update(angle)
    time.sleep(1)
    
    while True:
        angle += step
        
        if angle > 85: angle = -85
        
        print ("%.0f"%angle)
        servo_angle.update(angle)
        
        if angle == -85:
            time.sleep(1)
        time.sleep(sleep)
        
    """
    while True:
        print ("-45")
        servo_angle.update(-45)
        time.sleep(2)
        print ("0")
        servo_angle.update(0)
        time.sleep(2)
        print("+45")
        servo_angle.update(45)
        time.sleep(2)
=======
"""
Servoblaster from https://github.com/richardghirst/PiBits/tree/master/ServoBlaster

 Servo number    GPIO number   Pin in P1 header
          0               4             P1-7
          1              17             P1-11
          2              18             P1-12
          3             21/27           P1-13
          4              22             P1-15
          5              23             P1-16
          6              24             P1-18
          7              25             P1-22
          
Change the /etc/init.d/servoblaster file to OPTS="--idle-timeout=2000 --pcm --p1pins=<list>"

You can set the actual pin you want to dedicate to be servo with --p1pins option

"""

class ServoBlaster():
    _servo_dict = { 4: 0, 
                    17: 1,
                    18: 2,
                    21: 3,
                    22: 4,
                    23: 5,
                    24: 6,
                    25: 7}
                    
    def __init__(self, gpio_port):
        
        self._gpio_port     = gpio_port
        self._servo_number  = self._servo_dict[gpio_port]

    def update(self, duty_us):
        servoStr ="%u=%u\n" % (self._servo_number, long(duty_us*0.1))
        with open("/dev/servoblaster", "wb") as f:
            f.write(servoStr) 
            
class ServoAngle():
    def __init__(self, gpio_port, angle_min, angle_max, min_duty, max_duty):
        self._gpio_port = gpio_port
        
        self._angle_to_duty = (max_duty - min_duty)/(angle_max - angle_min)
        self._duty_min        = min_duty
        self._angle_min        = angle_min
        self._angle_max        = angle_max
        
        self._angle = 0
        
        self._servo            = ServoBlaster(gpio_port)
        
    def angle_to_duty(self, angle):
        duty = (angle - self._angle_min)*self._angle_to_duty + self._duty_min
        return (duty)
        
    def update(self, angle):
        self._servo.update(self.angle_to_duty(angle))
        self._angle = angle
        
    def set_to_min(self):
        self.update(self._angle_min)
        
    def set_to_max(self):
        self.update(self._angle_max)    
        
    def set_to_middle(self):
        self.update((self._angle_min + self._angle_max)*0.5)   

    @property
    def angle(self):
        return(self._angle)
        
    @property
    def angle_min(self):
        return(self._angle_min)
        
    @property
    def angle_max(self):
        return(self._angle_max)        
    
        
if __name__ == "__main__":
    import time
    #-- ROS Convention: counter clockwise is positive (left positive, right negative)
    servo_angle = ServoAngle(23, -85, 85, 2250, 750)
    
    angle = -85
    sleep = 0.01
    step = 2
    
    servo_angle.update(angle)
    time.sleep(1)
    
    while True:
        angle += step
        
        if angle > 85: angle = -85
        
        print ("%.0f"%angle)
        servo_angle.update(angle)
        
        if angle == -85:
            time.sleep(1)
        time.sleep(sleep)
        
    """
    while True:
        print ("-45")
        servo_angle.update(-45)
        time.sleep(2)
        print ("0")
        servo_angle.update(0)
        time.sleep(2)
        print("+45")
        servo_angle.update(45)
        time.sleep(2)
>>>>>>> 339226fe7f5a20562103b4bc3e9d6b0802dcd869
        """