import serial
import time
import numpy as np
import random

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

class PC_INTERFACE():
    def __init__(self,port ='COM5',baudrate=19200,timeout=.000001,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS):
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
        #self.arduino.write(bytes(x,'ascii'))
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
        fou_digit_v = '{0:04d}'.format(voltage)
        num = "3"+str(fou_digit_v)+"\r"
        print("SENDING:",num)
        #box.arduino.write(bytes(str(num),'ascii'))
        for i in num:
            box.arduino.write(bytes('{}'.format(i),'ascii'))
        #data = box.arduino.read()
        data = box.arduino.readline()
        #box.arduino.reset_input_buffer()
        #
        if data != b'':
            str_ret = bytes.decode(data)
            print("Recieved:",data," ",len(str_ret)," ",len(data))
            if(len(str_ret) <= 3):
                int_ret = int(str_ret)
                if(int_ret > 0 and int_ret < 1000):
                    return int_ret
        return 0

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
voltage_values = []
starttime = time.time()

nukber = 1
"""
while True:
    num = "3"+str(nukber)+"2"+"\r"
    #box.arduino.write(bytes(str(num),'ascii'))
    for i in num:
        box.arduino.write(bytes('{}'.format(i),'ascii'))
    #data = box.arduino.read()
    data = box.arduino.readline()
    #box.arduino.reset_input_buffer()
    #
    if data != b'':
        print("Recieved:",data)
        nukber = nukber +1;
        if nukber >= 10:
            nukber = 0
    # 01010001
    # 01000101
"""
value = box.SetVoltage(1)
time.sleep(0.5)
value = box.SetVoltage(99)
time.sleep(0.5)
value = box.SetVoltage(0)
time.sleep(2)
for i in range(0,1023):

    set_val = random.randint(0, 1023)
    #set_val = i
    value = box.SetVoltage(set_val)
    pwm_values.append(set_val)
    voltage_values.append(int(value))

    time.sleep(0.01)
    
print("starttime",starttime,"DURATION: ",time.time() - starttime)
fig, ax1 = plt.subplots() 
plt.title("PRS per PWM input")
ax1.set_xlabel('time') 
ax1.set_ylabel('PWM', color = 'red') 
ax1.plot(pwm_values,voltage_values, color = 'red') 
#ax1.tick_params(axis ='y', labelcolor = 'red') 
  
# Adding Twin Axes

#ax2 = ax1.twinx() 
  
#ax2.set_ylabel('RPS', color = 'blue') 
#ax2.plot(voltage_values, speed_values, color = 'blue') 
#ax2.tick_params(axis ='y', labelcolor = 'blue') 
 
# Show plot

plt.show()
