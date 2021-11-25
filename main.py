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
Sim = Simulator([0,0])

def main():
    # load the pre-recorded data
    #data2 = cHelper.readFromCSV("sample_waypoints")
    data2 = cHelper.readFromCSV("sample_waypoints_2")
    data2 = cHelper.readFromCSV("sample_waypoints_3")
    #data2 = cHelper.readFromCSV("manual_recording")
    #data2 = data2[:350]

    C3 = DeePC_OSQP.Controller(data2,2,10,3,3)
    # data, T_ini, T_f , nr inputs, nr outputs
    C3.updateIOConstrains([0,-1,0],[1,1,1],[-10000,-10000,-180],[10000,10000,180])
    C3.updateControlCost_R([[3,0,0],[0,3,0],[0,0,1]]) # quadratic control effort cost matrix,
    C3.updateTrackingCost_Q([[1,0,0],[0,1,0],[0,0,1]]) # quadratic tracking error cost matrix
    C3.update_lambda_g(500) #the weight on the regularization of 푔
    C3.update_lambda_s(0.000000001) #weight on the softened initial condition constraint

    reference = [-10.0 ,20.0, 80.0]
    print("Reference Point is:",reference)
    C3.updateReferenceWaypoint(reference)
    C3.updateReferenceInput([0.2 ,0.0, 0.0])
    
    Sim.InitWorld()
    while(Sim.Initalized != True):
        if Sim.Initalized == False:
            Sim.CleanUpAllActors()
            exit()
            break

    #Sim.resetVehicle()
    tick = 0
    while(tick < 200):
        tick = tick + 1
        #start filling y_ini and u_ini
        if tick == 100:
            reference = [-10.0 ,30.0, 120.0]
            C3.updateReferenceInput([0.0 ,0.0, 1.0])    

        Sim.DrawReferencePoint(reference)
        #optim_control,prediction = controller.getOptimalControlSequence() # workd exepet for the equality constrain
        u,y,u_star,y_star,g = C3.getInputOutputPrediction()
            
        throttle_in = u_star[0][0]
        steer_in = u_star[0][1]
        brake_in = u_star[0][2]
        Sim.ControlVehicle(throttle_in,steer_in,brake_in,True)
        

        outputs = Sim.GetVehicleInfo()
        inputs = [throttle_in, steer_in,brake_in]
        C3.updateIn_Out_Measures(inputs,outputs,True)
        Sim.UpdateRecordingData(reference,inputs,outputs,y_star[0])
        Sim.DrawPredictionPoint(y_star[0])

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


