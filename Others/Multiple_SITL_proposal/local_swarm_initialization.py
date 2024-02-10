#Swarm initialization

import subprocess
import time

external_broker = "nan"; username = "nan"; password = "nan"; simulation_port = "nan"; #internal_broker_port = 1884 #No dejar vacio
num_drones = 1

for i in range(num_drones):

    # Comando para iniciar SITL para un dron
    argvs = [str(num_drones)]
    command = ["start", "cmd", "/k", "python", "init_sitl.py"] + argvs
    command_str = ' '.join(command)
    
    #print(len(argvs))
    subprocess.Popen(command_str, shell=True)

    # Inicia la simulación en segundo plano
    time.sleep(1)

while input("Press any key to stop simulation... "):
    pass