import glob
import os
import sys
import numpy as np
# taken from example code
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import martinCarlaLibrary as mCL
import carla
import time
import random

# a system for the car that manages its own detection/ movement forecast and trajectory forecast
# every Traffic participat has someting like this
# it has as input all VRUs it can detect in its vecinity with a certainty (assumption for now is everbody with different certainty)
global_prediction_list = []

class EgoVehicle(object):
    """docstring for EgoVehicle."""

    def __init__(self,world,cVehicle,**kwargs,):
        super(EgoVehicle, self).__init__()
        self.nearby_vrus = []  #stores a list of all nearby vru Vehicle classes
        self.reachable_participants = [] #stores a list of all nearby rachable Vehicle classes
        self.nearby_vru_data = [] # stores a list like this: [[id,x,y,velx,vely,velz],[id,x,y,velx,vely,velz],...] for all nearby vrus
                                  # this data is only about what I myself detected.

        self.true_nearby_vru_data = []# these are the true values. I, as a vehicle would never find out about these

        self.same_data_lists = [] # all the data about all the vrus that I share with self.reachable_participants
                                  # countains duplicates
        self.merged_prediction = []# contains data from self.same_data_lists added and averaged. only unique entries. for format see self.nearby_vru_data

        self.cVehicle = cVehicle
        self.kwargs = kwargs
        self.world = world
        self.same = []
        if kwargs.get("debug") is not None:
            self.debug = kwargs.get("debug")
        else:
            self.debug = False

        self.colour = carla.Color(r=random.randint(0,254), g=random.randint(0,254), b=random.randint(0,254))
        if kwargs.get("detectionDistance") is not None:
            self.detectionDistance = kwargs.get("detectionDistance")
        else:
            self.detectionDistance = 10

        if kwargs.get("reachDistance") is not None:
            self.reachDistance = kwargs.get("reachDistance")
        else:
            self.reachDistance = 30



    # this is simple and stupid and does not account for actual vision/Detection
    # I can not implement an acutal VRU detection so checking distance and ID will have to do for Now
    def calcDistance(self, location1,location2):
        temp = np.sqrt((location1.x - location2.x)**2 +(location1.y - location2.y)**2 )
        return temp


    def ask_reachables_for_common_vrus(self,smart_vehicle_list):
        self.same = []
        self.reachable_participants = []
        self.same_data_lists = []
        reachablecounter = 0
        for i in smart_vehicle_list:
            try:
                i.cVehicle.get_location() #HERE cVEHICLE BECAUSEE SMART_VEHICLE_LIST!

            except:
                
                continue

            try:
                self.cVehicle.get_location()  #HERE cVEHICLE BECAUSEE THATS SELF!

            except:
                continue

            dist = self.calcDistance(i.cVehicle.get_location(),self.cVehicle.get_location())

            if (i != self and dist < self.reachDistance): # and it is a participant
                reachablecounter += 1
                self.reachable_participants.append(i)

        ## TODO: can save some time here by checing directly and not appending first
        if len(self.reachable_participants) == 0:
            return
        for other in self.reachable_participants:
            self.same = set(self.nearby_vrus) & set(other.nearby_vrus)
            # that is all vrus that I detect  and my reachable vehicles also detect

            if self.same != []:
                for s in self.same:
                    dist = self.calcDistance(s.get_location(),other.cVehicle.get_location())
                    self.same_data_lists.append(self.get_pos_vel_of_Vehicle(dist,s))

                    if self.debug == True:
                        location = s.get_location()
                        location.z = location.z + 2
                        rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
                        size = carla.Vector3D(x=0.1,y=0.1,z=0.1)
                        box = carla.BoundingBox(location,size)
                        self.world.debug.draw_string(location,str(s.id), color=carla.Color(r=0,g=254,b=254), life_time=0.01)
                        pass


    def get_merged_vru_prediction(self):

        self.merged_prediction = []
        if self.same_data_lists != []:
            A = np.array(self.same_data_lists)

            unq, unq_inv,counts = np.unique(A[:, 0], return_inverse=True,return_counts = True)
            out = np.zeros((len(unq), A.shape[1]), dtype=A.dtype)
            out[:, 0] = unq
            np.add.at(out[:, 1:], unq_inv, A[:, 1:])

            for x in range(0,len(out)):
                out[x] = np.divide(out[x],counts[x])
                out[x][0] = int(out[x][0]*counts[x])
            self.merged_prediction = out.tolist()

        return self.merged_prediction



    def get_pos_vel_of_Vehicle(self,distance,vehicle):
        distortion = self.get_detection_accuaracy(distance)
        temp = []
        temp.append(vehicle.id)
        temp.append(distortion)
        temp.append(vehicle.get_location().x*distortion)
        temp.append(vehicle.get_location().y*distortion)
        temp.append(vehicle.get_velocity().x*distortion)
        temp.append(vehicle.get_velocity().y*distortion)
        temp.append(vehicle.get_velocity().z*distortion)
        return temp

    def get_true_pos_vel_of_Vehicle(self,vehicle):
        temp = []
        temp.append(vehicle.id)
        temp.append(1)
        temp.append(vehicle.get_location().x)
        temp.append(vehicle.get_location().y)
        temp.append(vehicle.get_velocity().x)
        temp.append(vehicle.get_velocity().y)
        temp.append(vehicle.get_velocity().z)
        return temp


    def print_merged_data(self):
        if len(self.nearby_vrus) != 0:
           
            print( str(self.merged_prediction))

    def print_my_vru_prediction(self):
        if len(self.nearby_vrus) != 0:

            print( str(self.nearby_vru_data))

    def print_true_values(self):
        if len(self.nearby_vrus) != 0:

            print( str(self.true_nearby_vru_data))




    # mapping
    # the vehilce distance to a prediction value
    def get_detection_accuaracy(self,distance,alpha = 0.5,certainty = 0.02):
        if self.detectionDistance != 0:
            facor_r = distance / self.detectionDistance
        else:
            facor_r = 1
        facor = 1- facor_r*alpha
        precision_detection = np.abs(np.random.normal(loc=facor, scale=certainty, size=None))
        #precision_detection = np.abs(np.random.normal(loc=facor, scale=certainty*3*facor_r**2, size=None))
        if precision_detection>1.0:
            precision_detection = 1-np.abs((1-precision_detection))
        return precision_detection


    def nearby_VRU_detection(self,world,traffic_participants_list,certainty=0.09):
        self.nearby_vrus = [] # this is an array of the vehivles. It ges passed to the VRUSystem
        self.nearby_vru_data = []
        self.true_nearby_vru_data = []
        for i in traffic_participants_list:
            try:
                i.get_location()

            except:
                continue

            try:
                self.cVehicle.get_location()

            except:
                continue




            dist = self.calcDistance(i.get_location(),self.cVehicle.get_location())
            if(i != self and  dist < self.detectionDistance and ((i.type_id == "vehicle.diamondback.century") or
                                                                                    (i.type_id == "walker.pedestrian.0005"))):
                # determining the detection of a vru (assumingly all vrus get detected)


                detected = self.get_pos_vel_of_Vehicle(dist,i)
                true = self.get_true_pos_vel_of_Vehicle(i)
                self.true_nearby_vru_data.append(true)

                self.nearby_vrus.append(i)
                self.nearby_vru_data.append(detected)


                # draw a line to detected vrus
                if self.debug == True:
                    world.debug.draw_line(self.cVehicle.get_location(),i.get_location(),color=self.colour,life_time=0.05)

        # draw a box above myself :D
        if self.debug == True:
            location = self.cVehicle.get_location()
            location.z = location.z + 4
            rotation = carla.Rotation(pitch=0.0, yaw=45.0, roll=45.0)
            size = carla.Vector3D(x=0.5,y=0.5,z=0.5)
            box = carla.BoundingBox(location,size)
            world.debug.draw_box(box,rotation, thickness=0.1, color=self.colour, life_time=0.05)
