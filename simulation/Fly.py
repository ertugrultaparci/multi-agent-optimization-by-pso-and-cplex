import os

import airsim
import time
import json
from ast import literal_eval

from model.Swarm import swarm
from model.model_definition.Instance import Instance
from model.Optimization import Optimization

import simulation.simulation_multidrone as simulation
import formation

instance = Instance()
instance.readInstance("C:/Users/ertug/OneDrive/Masaüstü/swarm2022-main/model/model_input/Agents.csv")
optimization = Optimization()
# "model/model_input/Agents.csv").replace('\\', '/')
swarm(instance, optimization)

# Let's open simulation environment:
time.sleep(2)
os.startfile("D:/AirSim Packages/MSBuild2018/WindowsNoEditor/MSBuild2018.exe") # Open any program, text or office document
time.sleep(5)

client = airsim.MultirotorClient()
client.confirmConnection()

f_list = []

simulation.takeoff(client, instance, f_list)

time.sleep(1)

simulation.moveUp(client, instance, f_list)

time.sleep(1)

simulation.waitASecond(client, instance, f_list)

time.sleep(1)

formation.pid_formation(client, instance)

time.sleep(1)

simulation.waitASecond(client, instance, f_list)

time.sleep(1)

# Opening JSON file
with open("C:/Users/ertug/OneDrive/Masaüstü/swarm2022-main/model/model_output/visitedCell.txt") as f:
    contents = f.readlines()

visitedCell = literal_eval(contents[0])

for k, v in visitedCell.items():
    print(f'Time is : {k} and cells : {v}')
    simulation.moveForIteration(client, instance, 8, f_list, v)
    time.sleep(1)
    simulation.waitASecond(client, instance, f_list)
    time.sleep(1)

# simulation.moveInitialCell(client, instance, f_list, t=20)

# simulation.waitASecond(client, instance, f_list)


# simulation.scanArea(client, instance, f_list, t=15)

# simulation.waitASecond(client, instance, f_list)

simulation.moveBasePosition(client, instance, f_list, t=15)