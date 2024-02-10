import subprocess
import sys
from dronekit import connect, VehicleMode
from dronekit_sitl import SITL, start_default


def init_local_sitl(num_drones):

    base_port = "5762"
    num_drones = int(num_drones)
    for i in range(num_drones):

    #for j in range(num_drones): mas de un dron

        simulation_port = str(int(base_port) + 10*(i))  #i+j solo se usa porque son misma maquina donde se simula en el caso real: i(todo el bucle de j sobra)
        # Comando para iniciar SITL en el puerto especificado
    
        sitl = SITL()
        sitl.download("plane", "3.3.0", verbose=True)
        sitl.launch([], await_ready=True, restart=True, verbose=True)
        print(f"SITL iniciado en el puerto {simulation_port}.")
        
        droneId = f"drone{i}"
        connection_mode = "local" ; operation_mode = "simulation"; internal_broker = str(1884 + i); username="nan"; password="nan"

        argvs = [connection_mode, operation_mode, internal_broker, username,password, droneId, simulation_port, str(i)]
        command = ["start", "cmd", "/k", "python", "AutopilotService.py"] + argvs
        command_str = ' '.join(command)
        
        #print(len(argvs))
        subprocess.Popen(command_str, shell=True)


    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(1)

    num_drones = sys.argv[1]
    init_local_sitl(num_drones)

