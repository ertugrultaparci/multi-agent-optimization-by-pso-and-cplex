import airsim
import time

from model.Optimization import Optimization
from model.model_definition.Instance import Instance
from simulation.formation import drone_state
from simulation.simulation_multidrone import moveACellSingleDrone

client = airsim.MultirotorClient()
client.confirmConnection()

instance = Instance()
instance.readInstance("C:/Users/ertug/OneDrive/Masaüstü/multi-agent-optimization-by-pso-and-cplex/model/model_input/Agents.csv")
optimization = Optimization()
optimization.Stage1(instance)
optimization.Stage2(instance)

vehicle = instance.Agents[0].getName()
client.enableApiControl(True, vehicle_name=vehicle)
client.armDisarm(True, vehicle_name=vehicle)

print('base pos:', instance.Agents[0].getBasePosition())
print('İnitial Pos: with drone_state:', drone_state(client, instance.Agents[0]))

print(client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position)
f = client.takeoffAsync(vehicle_name=vehicle)
f.join()

print(client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
print("stabil")

f = client.moveByVelocityAsync(0, 0, -50, 5, vehicle_name=vehicle)
f.join()
time.sleep(1)
print(client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position)

f = moveACellSingleDrone(client, instance, 10, 9, instance.Agents[0], -50)
f.join()
time.sleep(1)

print('Cell0:  ', client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position)
print('Cell0 with drone_state:', drone_state(client, instance.Agents[0]))

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = client.moveByVelocityZAsync(0, 0, -50, 10, vehicle_name=vehicle)
f.join()
time.sleep(1)
print('80:   ', client.getMultirotorState(vehicle_name=vehicle).kinematics_estimated.position)

print(instance.Agents[0].getName())

"""
f = moveACellSingleDrone(client, instance, 10, 0, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)


f = moveACellSingleDrone(client, instance, 10, 9, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)


f = moveACellSingleDrone(client, instance, 10, 9, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(5)


f = moveACellSingleDrone(client, instance, 10, 90, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(5)

f = moveACellSingleDrone(client, instance, 10, 99, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(5)


f = moveACellSingleDrone(client, instance, 10, 99, instance.Agents[0], -50)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(5)
"""

"""


f = moveACellSingleDrone(client, instance, 10, 9, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)



f = moveACellSingleDrone(client, instance, 10, 10, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 19, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 20, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 29, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 30, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 39, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)
f = moveACellSingleDrone(client, instance, 10, 40, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 49, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 50, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 59, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)




f = moveACellSingleDrone(client, instance, 10, 60, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 69, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)


f = moveACellSingleDrone(client, instance, 10, 70, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 79, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 80, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 89, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 90, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

f = moveACellSingleDrone(client, instance, 10, 99, instance.Agents[0], -20)
f.join()
time.sleep(1)

f = client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name=vehicle)
f.join()
time.sleep(1)

"""