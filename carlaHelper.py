import carla
import random


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
def spawn_car(actor_list,world,x=0,y=0,**kwargs):
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

def saveAsCSV(filename,data):
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
def drawWaypoints(waypoints_file,world,colour,life_time):
    #waypoints = readFromCSV(waypoints_file)

    #print(waypoints)
    debug = world.debug
    for point in waypoints_file:
        debug.draw_box(carla.BoundingBox(carla.Location(point[0],point[1],3),carla.Vector3D(0.3,0.3,0.3)), carla.Rotation(0,0,0),0.05, carla.Color(colour[0],colour[1],colour[2],0),life_time)
    
def recordData(client,world,car,tickrate = 50):
    sample_waypoints = []
    try:
        while(1):
            if (world.tick() % tickrate == 0):
                control = car.get_control()
                transform = car.get_transform()

                #print(transform.location.x,transform.location.y,transform.rotation.yaw,control.throttle, control.steer, control.brake)
                sample_waypoints.append([transform.location.x,transform.location.y,transform.rotation.yaw,control.throttle, control.steer, control.brake])
                
    
    except KeyboardInterrupt:
        saveAsCSV("sample_waypoints",sample_waypoints)
        return
