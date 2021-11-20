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
import matplotlib.pyplot as plt
import DeePC_control as DeePCC
import DeePC_control_2 as DeePCC2
import DeePC_OSQP as DeePC_OSQP

actor_list = [] # store everything that has to be destroyed

def truncateInouts(throttle,steer):
    ret_throttle = throttle
    ret_steer = steer
    if throttle > 1:
        ret_throttle = throttle
    if throttle < 0:
        throttle = 0
    if steer > 1:
        ret_steer = 1
    if steer < -1:
        steer = -1
    return ret_throttle,ret_steer

def spawn_car(world,x=0,y=0,**kwargs):
        global actor_list
        cars = []
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle.Tesla.model3'))
        if kwargs.get("spawnRandom") is True:
            transform = random.choice(world.get_map().get_spawn_points())

        else:
            transform = world.get_map().get_spawn_points()[0]
            transform.location.y += y
            transform.location.x += x
            transform.location.z += 1

        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle is not None:
            print("spawning ",vehicle.type_id," ",vehicle.id," at: ", transform.location)

            actor_list.append(vehicle)
            return vehicle
        else:
            print("Returning none, nothing appended, no car spawnes")
            return None


# define waypoints
"""
#data2 = cHelper.readFromCSV("sample_waypoints")
data2 = cHelper.readFromCSV("sample_waypoints_2")
#data2 = cHelper.readFromCSV("TestData")
#data2 = cHelper.readFromCSV("manual_recording")
data2 = np.array(data2)
data2 = data2[:100,:]

C3 = DeePC_OSQP.Controller(data2, 5, 5, 3, 3)
C3.updateReferenceWaypoint([10,-70,0])
C3.test_erg()

exit()
C3.solve_for_x_regularized()
u,y,u_star,y_star,g = C3.getInputOutputPrediction()
print("u : ",u)
print("u*: ",u_star)
print("y : ",y)
print("y*: ",y_star)
#print("g : ",g)

exit()
"""

def main():
    global actor_list

    try:
        client = None
        client = carla.Client('localhost', 2000)

        tm = client.get_trafficmanager(8000)
        tm_port = tm.get_port()
        client.set_timeout(3.0)
        world = client.get_world()

        settings = world.get_settings()
        settings.no_rendering_mode = False
        #settings.no_rendering_mode = True # Enable this to not Render
        world.apply_settings(settings)

        # load the pre-recorded data
        #data2 = cHelper.readFromCSV("sample_waypoints")
        data2 = cHelper.readFromCSV("sample_waypoints_2")
        #data2 = cHelper.readFromCSV("manual_recording")
        data2 = data2[:350]
        #data = [[i,i,i,-i,-i,-i] for i in range(30)] # [output, output, output, input,input,input] -> for debugging

        # init the controller
        #controller = DeePCC.Controller(data,5,6,3,3) # data, T_ini, T_f , nr inputs, nr outputs
        C3 = DeePC_OSQP.Controller(data2,3,25,3,3)
        # the reference point is y_r and also drawn in red on the track
        for i in range(12):
            #controller.updateInputOutputMeasures(controller.output_sequence[i],controller.input_sequence[i])
            C3.updateIn_Out_Measures(C3.input_sequence[i], C3.output_sequence[i])


        temp,starting_transform = cHelper.spawn_car(actor_list,world,10,-60)

        starting_x = starting_transform.location.x
        starting_y = starting_transform.location.y
        offset = [starting_x,starting_y,0,0]
        print("Starting Pos of Car:",starting_x,starting_y)
        reference = [-10.0 + starting_x ,10.0 + starting_y, 0.0]
        C3.updateReferenceWaypoint(reference)
        cHelper.drawWaypoints([reference],world,[255,0,0],20.0)
        for v in actor_list:
            v.set_autopilot(True,tm_port)
        
         #tm.ignore_lights_percentage(temp,100) # use this only when record data. It makes the car ignore all red lights
        sample_waypoints = []

        #cHelper.drawWaypoints(data,world,[10,0,255],0.0)
        time.sleep(2)
        tick = 0
        while(1):
            tick = tick + 1

            control = temp.get_control()
            throttle_in = control.throttle
            steer_in = control.steer
            #start filling y_ini and u_ini             
            if tick > 20:
                #print("DeePC Controller Taking over:")
                time.sleep(0.1)
                for v in actor_list:
                    v.set_autopilot(False,tm_port)
                
                #optim_control,prediction = controller.getOptimalControlSequence() # workd exepet for the equality constrain
                u,y,u_star,y_star,g = C3.getInputOutputPrediction(True)

                cHelper.drawWaypoints(u_star,world,[0,0,255],1.0)
                #figure out longitudal control

                #figure out lateral control

                # apply control
                throttle_in, steer_in = truncateInouts(u_star[0][0],u_star[0][1])

                temp.apply_control(carla.VehicleControl(
                                    throttle = throttle_in,
                                    steer = steer_in,
                                    brake = 0.0,
                                    hand_brake = False,
                                    reverse = False,
                                    manual_gear_shift = False,
                                    gear = 0))

            transform = temp.get_transform()
            outputs = [transform.location.x-starting_x,transform.location.y-starting_y,transform.rotation.yaw]
            

            inputs = [throttle_in, steer_in, 0.0]
            C3.updateIn_Out_Measures(inputs,outputs,False)


            
        #cHelper.recordData(client,world,temp)

    except KeyboardInterrupt:
        print("Simulation terminated by Hand")
        
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        actor_list.clear()
        print('done.')

    finally:
        pass

        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        actor_list.clear()
        print('done.')


if __name__ == '__main__':

    main()


