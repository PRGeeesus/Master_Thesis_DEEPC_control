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
Sim = Simulator([10,260],3,5)

def main():
    # load the pre-recorded data
    #data2 = cHelper.readFromCSV("sample_waypoints")
    #data2 = cHelper.readFromCSV("sample_waypoints_2")
    #data2 = cHelper.readFromCSV("sample_waypoints_straight_2D")
    data2 = cHelper.readFromCSV("sample_waypoints_3")
    #data2 = cHelper.readFromCSV("manual_recording")
    #data2 = data2[:250]
    T_ini = 4
    T_f = 30
    settings = {"lambda_s":100,
                "lambda_g":100,
                "out_constr_lb":[-100,-100,-100,-100,-np.pi/2],
                "out_constr_ub":[100,100,100,100,np.pi/2],
                "in_constr_lb":[0.18,-0.7,0],
                "in_constr_ub":[0.20,0.7,1]}
    print("Data length:", len(data2))
    C3 = DeePC_OSQP.Controller(data2,T_ini,T_f,3,5,**settings)


    reference = [4,15,0,0,0.8] #realtice to car spawn
    print("Reference Point is:",reference)
    C3.updateReferenceWaypoint(reference)
    C3.updateControlCost_R([[1,0,0],[0,8,0],[0,0,100]])
    #C3.updateTrackingCost_Q([[1,0,0],[0,1,0],[0,0,1]])
    C3.updateTrackingCost_Q([[0.1,0,0,0,0],
                             [0,0.1,0,0,0],
                             [0,0,0.1,0,0],
                             [0,0,0,0.1,0],
                             [0,0,0,0,1000]])

    
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
    while(tick < 20000):

        tick = tick + 1
        #start filling y_ini and u_ini
        Sim.DrawReferencePoint(reference)
        #optim_control,prediction = controller.getOptimalControlSequence() # workd exepet for the equality constrain
        u,y,u_star,y_star,g = C3.getInputOutputPrediction()
            
        throttle_in = u[0][0]
        steer_in = u[0][1]
        #brake_in = u_star[0][2]
        brake_in = 0
        Sim.ControlVehicle(throttle_in,steer_in,brake_in,False)
        

        outputs = Sim.GetVehicleInfo()
        #outputs = [outputs[1]+140.0]
        inputs = [throttle_in, steer_in,brake_in]
        #inputs = [throttle_in]
        C3.updateIn_Out_Measures(inputs,outputs)
        
        print("Controls:",['{:.2f}'.format(i) for i in inputs],
              " Prediction: ",['{:.2f}'.format(i) for i in y[0]],
              " Outputs: ",['{:.2f}'.format(i) for i in outputs])

        Sim.UpdateRecordingData(reference,inputs,outputs,y[0])
        #print(y[:,:2])
        Sim.DrawPredictionPoint(y[:,:2])

        #print("Dist to point:",np.sqrt((outputs[0]-reference[0])**2 + (outputs[1]-reference[1])**2))
        if np.sqrt((outputs[0]-reference[0])**2 + (outputs[1]-reference[1])**2) < 3:
            print("Close enough")
            reference = [8,20,0,0,0]
            C3.updateReferenceWaypoint(reference)

    print("Siumlation done by ticks")
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
    


