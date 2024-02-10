#Swarm initialization
#Need previously a missionplanner SITL initialized

import subprocess
import time

external_broker = "classpip_cred"
username = "dronsEETAC" ; password = "mimara1456."

base_port = "5762"

num_drones = 4
drones_cruise_velocity = '3'
conflictRadius = '20'
link_Distance = '100'
safe_factor = '2'
delta_t = '1'

#Cal alguna manera easy de inixializarlos junto a sus prios
procesos_sitl = []; log_Flight_Plan_Flag = '1' 

for i in range(num_drones):
    
    simulation_port = str(int(base_port) + 10*i)
    droneId = f"drone{i}"
    connection_mode = "global" ; operation_mode = "simulation"

    argvs = [connection_mode, operation_mode, external_broker, username,password, 
             droneId, simulation_port, str(i), log_Flight_Plan_Flag,
             drones_cruise_velocity,conflictRadius,link_Distance,safe_factor,delta_t]
    command = ["start", "cmd", "/k", "python", "AutopilotService_V_0_3_4.py"] + argvs
    command_str = ' '.join(command)
    
    #print(len(argvs))
    subprocess.Popen(command_str, shell=True)
    time.sleep(2)

while input("Press any key to stop simulation... "):
    pass