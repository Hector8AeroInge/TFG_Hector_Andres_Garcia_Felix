import subprocess
import sys

def init_local_sitl(num_drones):

    base_port = "5760"
    port_factor = 4 #Factor added to unable otrher unused ports becaus sitl-dronekit does not have the option to intialize on specified port
    num_drones = int(num_drones)
    for i in range(num_drones):

    #for j in range(num_drones): mas de un dron
        
        simulation_port = str(int(base_port) + 10*(i+port_factor))  #i+j solo se usa porque son misma maquina donde se simula en el caso real: i(todo el bucle de j sobra)
        # Comando para iniciar SITL en el puerto especificado
        comando = f'dronekit-sitl copter --model quad --instance {i+port_factor} --home=-35.363261,149.165230,584,353'  #--console
        
        # Inicia SITL en una nueva consola
        subprocess.Popen(comando, shell=True)
        print(f"SITL iniciado en el puerto {simulation_port}.")
        
        droneId = f"drone{i}"
        connection_mode = "local" ; operation_mode = "simulation"; internal_broker = str(1884 + i); username="nan"; password="nan"

        print("AutopilotService_V_0_3_1.py")
        argvs = [connection_mode, operation_mode, internal_broker, username,password, droneId, simulation_port, str(i)]
        command = ["start", "cmd", "/k", "python", "AutopilotService_Local_V_0_3_1.py"] + argvs
        command_str = ' '.join(command)
        
        #print(len(argvs))
        subprocess.Popen(command_str, shell=True)


    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(1)

    num_drones = sys.argv[1]
    init_local_sitl(num_drones)

