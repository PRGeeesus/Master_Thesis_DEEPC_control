import serial
import time
import numpy as np
import random

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

class PC_INTERFACE():
    def __init__(self,port ='COM3',baudrate=115200,timeout=.00001):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.arduino = serial.Serial(port = self.port, baudrate = self.baudrate,timeout = self.timeout)
        #self.arduino = serial.Serial(port = self.port, baudrate = self.baudrate)
        time.sleep(3)
        print("Connection Estabished")
        self.verbose = True
        self.escs_pwm = 1200
    
        self.time = time.time()
        self.prev_time = 0
        self.prev_pos = 0
        self.pos = 0
        self.prev_rpm = 0
        self.rpm = 0
        self.cc_range = [i for i in range(1315,1600,1)]

    def write_read(self,x):
        self.arduino.write(bytes(x, 'utf-8'))
        #time.sleep(0.05)
        data = self.arduino.readline()
        return data
    
    def SetAnswer(self,a0,a1,echo):
        payload = str(0)+str(a0)+str(a1)+str(echo)+str('\n')
        value = self.write_read(payload)
        if self.verbose:
            print("Setting Settings:\n")
            print("Answer voltage a0:"+str(a0)+"\n")
            print("Answer voltage a1:"+str(a1)+"\n")
            print("Answer echo payload:"+str(echo)+"\n")
            print("Return:\n" + str(value)+"\n")
        
    
    def SetVoltage(self,voltage):
        payload = str(3)+str(voltage)+str('\n')
        value = self.write_read(payload)

        if self.verbose: print("Setting Voltage: Mode:3 voltage: "+ str(voltage) + " Answer:" + str(value)) # printing the value
        return value

    def ESC_RW(self):
        self.prev_pos = self.pos
        self.pos = 0
        payload = str(3)+str(self.escs_pwm)+str('\n')
        value = self.write_read(payload)
        raw_str = str(value.decode('utf-8'))
        self.prev_rpm = self.rpm
        #if self.verbose: print("string",raw_str[:len(raw_str)-2]," len: ",len(raw_str))
        if len(raw_str) == 8 and raw_str[1] == ';':
            self.rpm = float(raw_str[2:len(raw_str)-2]) 
            if self.verbose: print("Setting ESC: Mode:3 escs_pwm: "+ str(self.escs_pwm) + " DIR: " + str(raw_str[0]) + " RPM:" +str(self.rpm)) # printing the value

        return self.rpm


box = PC_INTERFACE()

pwm_values = []
speed_values = []
x_values = []
starttime = time.time()

for i in range(99):
    
    #set_val = random.randint(0, 99)
    set_val = i
    value = box.SetVoltage(set_val)
    pwm_values.append(set_val)
    x_values.append(value)
    time.sleep(0.01)
    
print("starttime",starttime,"DURATION: ",time.time() - starttime)
fig, ax1 = plt.subplots() 
plt.title("PRS per PWM input")
ax1.set_xlabel('time') 
ax1.set_ylabel('PWM', color = 'red') 
ax1.plot(pwm_values, x_values, color = 'red') 
ax1.tick_params(axis ='y', labelcolor = 'red') 
  
# Adding Twin Axes

#ax2 = ax1.twinx() 
  
#ax2.set_ylabel('RPS', color = 'blue') 
#ax2.plot(x_values, speed_values, color = 'blue') 
#ax2.tick_params(axis ='y', labelcolor = 'blue') 
 
# Show plot

plt.show()
