import sys
sys.path.append('C:\\Users\\Martin\\Documents\\CARLA\\PythonAPI')
#import carla
import random
import time
import numpy as np

# Needed if using many actors
def checkingAllActorsAlive(actor_list,world):
    counter = 0
    world.get_actors([actor.id for actor in actor_list])
    if counter > 1:
        print(counter," / ",len(actor_list)," were faulty!")

    for a in actor_list:
        try:
            a.is_alive
        except:
            print("Actor", a.id , " not alive!")

# use to manage actor lifecycle
def spawn_car(world,x=0,y=0,**kwargs):
        cars = []
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle.Tesla.model3'))
        if kwargs.get("spawnRandom") is True:
            transform = random.choice(world.get_map().get_spawn_points())

        else:
            transform = world.get_map().get_spawn_points()[0]
            transform.location.y += y
            transform.location.x += x
            transform.location.z += 1.0
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle is not None:
            print("spawning ",vehicle.type_id," ",vehicle.id," at: ", transform.location)

            return vehicle,transform
        else:
            print("Returning none, nothing appended, no car spawnes. Check if you deleted the previous Car")
            return None

def saveAsCSV(filename,data):
    print(len(data)," datapoints collected")
    print("Saving recorded data in csv file: " + filename + ".csv")
    with open(filename + '.csv', 'w') as f:
        # using csv.writer method from CSV package
        for line in data:
            string = str(line).replace('[','')
            string = string.replace(']','')
            f.write(string + "\n")

def readFromCSV(filename):
    with open(filename + '.csv', 'r') as f:
        # using csv.writer method from CSV package
        temp = []
        for line in f:
            single_line = line.split(',')
            sub = []
            for entry in single_line:
                sub.append(float(entry))
            temp.append(sub)
        return temp

#draw waypoints as rectangles on the map
def drawWaypoints(waypoints_file,world,colour,life_time,xyoffset = [0.0,0.0]):
    #waypoints = readFromCSV(waypoints_file)

    #print(waypoints)
    debug = world.debug
    for point in waypoints_file:
        
        debug.draw_box(carla.BoundingBox(carla.Location(point[0] + xyoffset[0],point[1]+xyoffset[1],3),carla.Vector3D(0.3,0.3,0.3)), carla.Rotation(0,0,0),0.05, carla.Color(colour[0],colour[1],colour[2],0),life_time)

def generateTestData(name,dataset_lengt = 350,datapoint_length = 6):
    #TestData = [[0.1,0.2,0.3,0.4,0.5,0.6]]
    TestData = [[0.1,0.2,0.3,0.4,0.5,0.6]]

    for i in range(1,dataset_lengt):
        TestData.append([0.1+i,0.2+i,0.3+i,0.4+i,0.5+i,0.6+i])
    saveAsCSV(name, TestData)

def deductStartingPositionFromData(data,x_offset,y_offset):
    for i in data:
        data[0] = data[0]-x_offset
        data[1] = data[1]-y_offset

def recordData(client,world,car,name = "sample_waypoints",sample_interval_ms = 40,nr_samples = None):
    """
    recors data in the way:
    [throttle,steer,brake,car-x,car-y,car-yaw]
    therefore it is:
    [u1,u2,u3, y1,y2,y3]
    """
    sample_waypoints = [[0,0,0,0,0,0]]
    nr_datapoints = 0
    sample_interval_ns = sample_interval_ms*1000*1000
    running = True
    time.sleep(2)
    print("Begin Recording:")
    starting_pos = car.get_transform()
    starting_x = starting_pos.location.x
    starting_y = starting_pos.location.y
    prev_timestamp = 0
    start_time_ns = time.time_ns()

    try:
        while(running):
            time_ns = time.time_ns()
            # milli -> micro -> nano
            if time_ns > start_time_ns + nr_datapoints * sample_interval_ns and prev_timestamp != time_ns:
                #print(time_ns," % ",sample_interval_ms*1000*1000," = ",time_ns % sample_interval_ms*1000*1000)
                control = car.get_control()
                transform = car.get_transform()
                valocity = car.get_velocity()
                if sample_waypoints[-1][0] != transform.location.x and sample_waypoints[-1][1] != transform.location.y:
                    data = [control.throttle,
                            control.steer,
                            control.brake,
                            transform.location.x-starting_x,
                            transform.location.y-starting_y,
                            valocity.x,
                            valocity.y,
                            transform.rotation.yaw/180*np.pi]
                    print(nr_datapoints," ",time_ns - prev_timestamp," Recoding: ",data)
                    #print(transform.location.x,transform.location.y,transform.rotation.yaw,control.throttle, control.steer, control.brake)
                    sample_waypoints.append(data)
                    nr_datapoints = nr_datapoints + 1
                prev_timestamp = time_ns
            if len(sample_waypoints) == nr_samples:
                running = False
        
        sample_waypoints.pop(0)
        saveAsCSV(name,sample_waypoints)

    except KeyboardInterrupt:
        sample_waypoints.pop(0)
        saveAsCSV(name,sample_waypoints)
        return
