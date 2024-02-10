#Single drone connection to MQTT and to Autopilot service
import subprocess

#First hardocde DroneID and drone mission planner simulator port
droneId = "drone0"
simulationPort = "5782"  #Port as {string}

#First we will connect as simulator until solution is reached
connection_mode = "global"
operation_mode = "simulation"
external_broker = "classpip_cred"
username = "dronsEETAC" ; password = "mimara1456."

argvs = [connection_mode, operation_mode, external_broker, username, password, droneId, simulationPort]
command = ["start", "cmd", "/k", "python", "AutopilotService.py"] + argvs
command_str = ' '.join(command)

subprocess.Popen(command_str, shell=True)

while True:
    pass
    



