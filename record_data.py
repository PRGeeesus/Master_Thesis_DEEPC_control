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
import random

actor_list = [] # store everything that has to be destroyed


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

#data = cHelper.readFromCSV("sample_waypoints")
# using this instead to better debug the matricies
#data = [[i,i,i,-i,-i,-i] for i in range(30)] # [output, output, output, input,input,input]
#scontroller = DeePCC.Controller(data,5,6,3,3) # data, T_ini, T_f , nr inputs, nr outputs



#for i in range(10): controller.updateInputOutputMeasures([random.random()+1,random.random()-100,0.0],[random.random()*0.5,random.random(),0.0])

#cost = controller.costfunction([2,20,0.2],[2,20,0.2],[0.0,0.0,0.0])
#temp = [[-0.28545455],[-0.21560606],[-0.14575758],[-0.07590909],[-0.00606061],[ 0.06378788],[ 0.13363636],[ 0.20348485],[ 0.27333333]]
#temp2 = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
#gr = controller.get_g_r(temp)
#control,prediction = controller.getOptimalControlSequence() # workd exepet for the equality constrain
#print(control[0])
#print(prediction[0])



def main():
    global actor_list

    try:

        #set up Cala server
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
        starttick = world.tick()

        starting_x = 10
        starting_y = -70
        temp,starting_transform = cHelper.spawn_car(world,starting_x,starting_y)
        tm.ignore_lights_percentage(temp,100) # use this only when record data. It makes the car ignore all red lights
        for v in actor_list:
            v.set_autopilot(True,tm_port)
        temp.set_autopilot(True,tm_port)
        
         #tm.ignore_lights_percentage(temp,100) # use this only when record data. It makes the car ignore all red lights
        sample_waypoints = []

        #cHelper.drawWaypoints(data,world,[10,0,255],0.0)
        time.sleep(2)     
        cHelper.recordData(client,world,temp,"sample_waypoints_3",20,400)
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        carla.command.DestroyActor(temp)
        
    except KeyboardInterrupt:
        print("Simulation terminated by Hand")
        
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        actor_list.clear()
        print('done.')

    finally:
        
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        
        actor_list.clear()
        print('done.')


if __name__ == '__main__':

    main()
