import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
vehicle = random.choice(blueprint_library.filter('vehicle.Tesla.model3'))
location = random.choice(world.get_map().get_spawn_points())
vehicle = world.try_spawn_actor(bp, transform)