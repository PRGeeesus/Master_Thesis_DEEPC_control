#!/usr/bin/env python
# This code was created by martin Buchschuster
"""
In order to get the Carla server up and running please refer to: https://carla.readthedocs.io/en/latest/start_quickstart/
The "CarlaUE4.exe" has to run and be adressable on port 2000 in order for the code to run.

"""
import glob
import os
import sys
import csv


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np
import random
import time
import martinCarlaLibrary
import carlaHelper as cHelper
import DeePC_OSQP as DeePC_OSQP

from VehicleSimulator import Simulator






# define waypoints
"""
#data2 = cHelper.readFromCSV("sample_waypoints")
data2 = cHelper.readFromCSV("sample_waypoints_2")
#data2 = cHelper.readFromCSV("TestDataSet")
#data2 = cHelper.readFromCSV("manual_recording")
data2 = np.array(data2)
#data2 = data2[:100,:]

C3 = DeePC_OSQP.Controller(data2, 2, 6, 3, 3)
#C3.updateIOConstrains([0,-1,0],[1,1,1],[-1,-1,-1],[1,1,1])
C3.updateIOConstrains([0,-1,0],[1,1,1],[-10000,-10000,-180],[10000,10000,180])
C3.updateReferenceWaypoint([10,-70,0.0])
#C3.update_lambda_g(0.0000001)
#C3.update_lambda_s(100)

u,y,u_star,y_star,g = C3.getInputOutputPrediction(verbose=True)

print(u[0],y[0],u_star[0],y_star[0])


exit()
"""
Sim = Simulator([0,0],1,1)

def main():
    # load the pre-recorded data
    #data2 = cHelper.readFromCSV("sample_waypoints")
    #data2 = cHelper.readFromCSV("sample_waypoints_2")
    data2 = cHelper.readFromCSV("sample_waypoints_straight_2D")
    #data2 = cHelper.readFromCSV("sample_waypoints_3")
    #data2 = cHelper.readFromCSV("manual_recording")
    #data2 = data2[:350]
    T_ini = 2
    T_f = 20
    settings = {"lambda_s":100,
                "lambda_g":1,
                "out_constr_lb":[-10,],
                "out_constr_ub":[100000],
                "in_constr_lb":[0],
                "in_constr_ub":[1]}

    C3 = DeePC_OSQP.Controller(data2,T_ini,T_f,1,1,**settings)


    reference = [40]
    print("Reference Point is:",reference)
    C3.updateReferenceWaypoint(reference)
    #C3.updateReferenceInput([0.2 ,0.0, 0.0])
    
    print("Initalize World:")
    Sim.InitWorld()
    while(Sim.Initalized != True):
        if Sim.Initalized == False:
            Sim.CleanUpAllActors()
            exit()
            break

    #Sim.resetVehicle()
    tick = 0
    brake_in = 0
    while(tick < 400):
        tick = tick + 1
        #start filling y_ini and u_ini
        Sim.DrawReferencePoint([reference[0],0])
        #optim_control,prediction = controller.getOptimalControlSequence() # workd exepet for the equality constrain
        u,y,u_star,y_star,g = C3.getInputOutputPrediction()
            
        throttle_in = u_star[0][0]
        #steer_in = u_star[0][1]
        #brake_in = u_star[0][2]
        #brake_in = 0
        Sim.ControlVehicle(throttle_in,0,brake_in,True)
        

        outputs = Sim.GetVehicleInfo()
        outputs_full = outputs
        outputs = [outputs[1]+140.0]
        #inputs = [throttle_in, steer_in,brake_in]
        inputs = [throttle_in]
        C3.updateIn_Out_Measures(inputs,outputs,True)
        Sim.UpdateRecordingData(reference,inputs,outputs,y_star[0][0])
        #print("check: ",inputs,outputs,y_star[0])
        Sim.DrawPredictionPoint([0,y_star[0][0]])
        if outputs[0] < 52 and outputs[0] > 49:
            brake_in = 1
            break


    Sim.plot_results()
    Sim.CleanUpAllActors()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("ctrl + c pressed:")
        Sim.CleanUpAllActors()
    except Exception as e:
        print("Failed:",e)
        Sim.CleanUpAllActors()
    


