import carla
import numpy as np
import carlaHelper as cHelper
import time
import matplotlib.pyplot as plt


class Simulator:
    def __init__(self,Vehicle_start_pt,input_size, output_size):
        
        self.actor_list = [] # store everything that has to be destroyed
        self.input_history = np.array([0 for i in range(input_size)])
        self.output_history = np.array([0 for i in range(output_size)])
        self.prediction_history = np.array([0 for i in range(output_size)])
        self.reference_history = np.array([0 for i in range(output_size)])

        self.Vehicle_start_point = Vehicle_start_pt
        self.Vehicle = None
        self.AutoPilotMode = True
        self.SetAutopilot(self.AutoPilotMode)
        self.Initalized = None
        self.client = None

    def InitWorld(self):
        try:
    
            self.client = carla.Client('localhost', 2000)

            tm = self.client.get_trafficmanager(8000)
            self.tm_port = tm.get_port()
            self.client.set_timeout(3.0)
            #world = client.load_world("Town03")
            self.world = self.client.get_world()

            settings = self.world.get_settings()
            settings.no_rendering_mode = False
            #settings.no_rendering_mode = True # Enable this to not Render
            self.world.apply_settings(settings)

            temp,starting_transform = cHelper.spawn_car(self.world,
                                                        self.Vehicle_start_point[0],
                                                        self.Vehicle_start_point[1])
            if temp is not None:
                self.actor_list.append(temp)

            self.starting_x = starting_transform.location.x
            self.starting_y = starting_transform.location.y
            self.Vehicle = temp
            self.offset = [self.starting_x,self.starting_y,0.0]
            print("Starting Pos of Car:",self.starting_x,self.starting_y)
            print("Corrected Starting Pos of Car:")
            self.start_time_ns = time.time_ns()
            self.Initalized = True

        except KeyboardInterrupt:
            print("Simulation terminated by Hand")
            print('destroying actors')
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
            self.actor_list.clear()
            print('done.')
            self.Initalized = None

        except Exception as e:
            print("Init Failed:",e)
            self.Initalized = False

    def CleanUpAllActors(self):
        #if self.actor_list  or self.client is not None:
        print('destroying actors')
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        self.actor_list.clear()
        print("Done")
            

    def ControlVehicle(self,throttle,steer,brake,verbose = False):
        if self.AutoPilotMode == True:
            self.SetAutopilot(False)
        if verbose:
            print("Controlling: ",round(throttle,2),round(steer,2),round(brake,2))
        self.Vehicle.apply_control(carla.VehicleControl(
                                throttle = throttle,
                                steer = steer,
                                brake = brake,
                                hand_brake = False,
                                reverse = False,
                                manual_gear_shift = False,
                                gear = 0))
    
    def GetVehicleInfo(self):
        transform = self.Vehicle.get_transform()
        velocity = self.Vehicle.get_velocity()
        #outputs = [transform.location.x-self.starting_x,
        #           transform.location.y-self.starting_y,
        #           transform.rotation.yaw]
        outputs = [transform.location.x-self.starting_x,
        transform.location.y-self.starting_y,
        velocity.x,
        velocity.y,
        (np.pi/2)*(transform.rotation.yaw/180)]
        return outputs


    def SetAutopilot(self,Mode):
        self.AutoPilotMode = Mode
        for v in self.actor_list:
            v.set_autopilot(self.AutoPilotMode,self.tm_port)



    def resetVehicle(self):
        print(type(self.Vehicle))
        self.Vehicle.set_target_velocity(carla.Vector3D(0.0,0.0,0.0))
        self.Vehicle.set_transform(carla.Transform(carla.Location(self.Vehicle_start_point[0],self.Vehicle_start_point[1],2.0)),
                                                    carla.Rotation(0.0,0.0,90.0))

    def DrawReferencePoint(self,reference):
        cHelper.drawWaypoints([reference],self.world,[255,0,0],1.0,self.offset)
    
    def DrawPredictionPoint(self,predictions):
        cHelper.drawWaypoints(predictions,self.world,[0,0,255],1.0,self.offset)

    def UpdateRecordingData(self,reference,inpuuut,output,prediction):
        #self.prediction_history = np.vstack((self.prediction_history,prediction))
        self.reference_history = np.vstack((self.reference_history,reference))
        self.input_history = np.vstack((self.input_history,inpuuut))
        self.output_history = np.vstack((self.output_history,output))

    def plot_results(self):
        np.delete(self.output_history, -1)
        np.delete(self.input_history, -1)
        np.delete(self.reference_history, -1)
        np.delete(self.prediction_history, -1)
        fig, ax1 = plt.subplots()
        titlesting = "Title"
        plt.title(titlesting)
        ax1.set_ylabel("Outputs")

        ax1.plot(self.output_history[:,0],label='x',linewidth=3,c="r")
        #ax1.plot(self.output_history[:,1],label='y',linewidth=3,c="c")
        ax1.plot(self.reference_history[:,0],label='x_ref',c="k",linewidth=3)
        #ax1.plot(self.reference_history[:,1],label='y_ref',c="b",linewidth=3)
        ax1.plot(self.prediction_history[:,0],label='predictions',c="r")
        #ax1.plot(outputs2,label="system behaviour",c="g")
        plt.legend(loc=8)
        # ...
        ax2 = ax1.twinx()
        ax2.set_ylabel("Inputs")
        ax2.plot(self.input_history[:,0],label='throttle',c="y",linewidth=3)
        #ax2.plot(self.input_history[:,1],label='steer',c="lime",linewidth=3)
        ax2.set_ylim([-2, 2])
        plt.legend(loc=4)
        plt.show()

    def spawn_car(self,world,x=0,y=0,**kwargs):
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

                self.actor_list.append(vehicle)
                return vehicle
            else:
                print("Returning none, nothing appended, no car spawnes")
                return None
        
    def truncateInouts(self,throttle,steer):
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