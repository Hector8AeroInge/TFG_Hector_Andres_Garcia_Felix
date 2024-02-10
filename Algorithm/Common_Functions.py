#   .----------------. .----------------. .----------------. .----------------. .----------------. .-----------------.                                                         
#  | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. |                                                         
#  | |     ______   | | |     ____     | | | ____    ____ | | | ____    ____ | | |     ____     | | | ____  _____  | |                                                         
#  | |   .' ___  |  | | |   .'    `.   | | ||_   \  /   _|| | ||_   \  /   _|| | |   .'    `.   | | ||_   \|_   _| | |                                                         
#  | |  / .'   \_|  | | |  /  .--.  \  | | |  |   \/   |  | | |  |   \/   |  | | |  /  .--.  \  | | |  |   \ | |   | |                                                         
#  | |  | |         | | |  | |    | |  | | |  | |\  /| |  | | |  | |\  /| |  | | |  | |    | |  | | |  | |\ \| |   | |                                                         
#  | |  \ `.___.'\  | | |  \  `--'  /  | | | _| |_\/_| |_ | | | _| |_\/_| |_ | | |  \  `--'  /  | | | _| |_\   |_  | |                                                         
#  | |   `._____.'  | | |   `.____.'   | | ||_____||_____|| | ||_____||_____|| | |   `.____.'   | | ||_____|\____| | |                                                         
#  | |              | | |              | | |              | | |              | | |              | | |              | |                                                         
#  | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' |                                                         
#   .----------------. .----------------. .-----------------..----------------. .----------------. .----------------. .----------------. .-----------------..----------------. 
#  | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. | .--------------. |
#  | |  _________   | | | _____  _____ | | | ____  _____  | | |     ______   | | |  _________   | | |     _____    | | |     ____     | | | ____  _____  | | |    _______   | |
#  | | |_   ___  |  | | ||_   _||_   _|| | ||_   \|_   _| | | |   .' ___  |  | | | |  _   _  |  | | |    |_   _|   | | |   .'    `.   | | ||_   \|_   _| | | |   /  ___  |  | |
#  | |   | |_  \_|  | | |  | |    | |  | | |  |   \ | |   | | |  / .'   \_|  | | | |_/ | | \_|  | | |      | |     | | |  /  .--.  \  | | |  |   \ | |   | | |  |  (__ \_|  | |
#  | |   |  _|      | | |  | '    ' |  | | |  | |\ \| |   | | |  | |         | | |     | |      | | |      | |     | | |  | |    | |  | | |  | |\ \| |   | | |   '.___`-.   | |
#  | |  _| |_       | | |   \ `--' /   | | | _| |_\   |_  | | |  \ `.___.'\  | | |    _| |_     | | |     _| |_    | | |  \  `--'  /  | | | _| |_\   |_  | | |  |`\____) |  | |
#  | | |_____|      | | |    `.__.'    | | ||_____|\____| | | |   `._____.'  | | |   |_____|    | | |    |_____|   | | |   `.____.'   | | ||_____|\____| | | |  |_______.'  | |
#  | |              | | |              | | |              | | |              | | |              | | |              | | |              | | |              | | |              | |
#  | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' | '--------------' |
#   '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' '----------------' 

import numpy as np
import classes
import datetime
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
paquete_dir = os.path.join(script_dir, '..', 'Swarm')
sys.path.append(paquete_dir)

import geo_functions

def reset_All_Drones(traffic):
    #**********************************************************************************************
    # Función la cual devuelve a todos los drones de la lista "traffic" a sus condiciones iniciales 
    #**********************************************************************************************
    for drone in traffic:

        drone.reset()
        

def set_path_as_waypoint_All_Drones(traffic):
    #********************************************************************************************
    # Función la cual actualiza la lista de waypoints de cada dron de la lista "traffics" por los
    # puntos que componen el recorrido previo de cada dron
    #********************************************************************************************
    for drone in traffic:

        drone.set_Path_As_Way_Points()


def reset_All_Paths(traffic):
    #***************************************************************************************************************
    # Función la cual resetea la lista de puntos que componen el recorrido previo de cada dron de la lista "traffic"
    #***************************************************************************************************************
    for drone in traffic:

        drone.path = []


def reset_all_drones_solution_waypoints(traffic):
    #************************************************************************************************
    # Función la cual resetea la lista de waypoints de la solución de cada dron de la lista "traffic"
    #************************************************************************************************
    for drone in traffic:

        drone.solution_waypoints = []
        

def reset_all_drones_solution_restrictions(traffic):
    #************************************************************************************************
    # Función la cual resetea la lista que contiene las restricciones de movimiento de la solución
    # de cada dron de la lista "traffic"
    #************************************************************************************************
    for drone in traffic:

        drone.restriction_solution = []

        
def distance_Between_Drones(drone0, drone1):
    #************************************************************************************************
    # Función la cual calcula la distancia absoluta entre dos drones en el espacio tridimensional
    #************************************************************************************************
    distance = np.linalg.norm(np.array([drone0.position.x - drone1.position.x, drone0.position.y - drone1.position.y, drone0.position.z - drone1.position.z]))
    return abs(distance)


def agregar_objeto_en_posicion(vector, objeto, posicion):
    #*****************************************************************************************************
    # Función para la cual dados un vector de listas y una posición, agrega un objeto al final de la lista
    # que se encuentre en la posición indicada
    #*****************************************************************************************************

    # Asegurarse de que el vector tenga al menos la longitud de la posición deseada
    while len(vector) <= posicion:

        vector.append(None)  # Agregar nulos (puedes usar otro valor predeterminado)
    
    # Agregar el objeto en la posición deseada
    try: 
        if objeto not in vector[posicion]:  

            vector[posicion].append(objeto) 

    except: vector[posicion] = [objeto]
        

def update_All_Drones_Path(traffic):

    #*****************************************************************************************************
    # Función para la cual dados un vector de listas y una posición, agrega un objeto al final de la lista
    # que se encuentre en la posición indicada
    #*****************************************************************************************************
    for drone in traffic:
        if drone.status != "ended":
            drone.path.append(drone.position)
    
def calculate_new_position(initialPoint, directionVector, scalarVelocity, delta_t):
    #*****************************************************************************************************
    # Función para la cual para un punto inicial, un vector de direcciónm unitario, una velocidad y un intervalo
    # de tiempo, calcula la posición después de ese intervalo de tiempo
    #*****************************************************************************************************
    new_x = initialPoint.x + (scalarVelocity * directionVector[0] * delta_t)
    new_y = initialPoint.y + (scalarVelocity * directionVector[1] * delta_t)
    new_z = initialPoint.z + (scalarVelocity * directionVector[2] * delta_t)

    return classes.Point(new_x, new_y, new_z)

def relative_Position_Between_Drones(drone0, drone1):
    #********************************************************************
    # Función para la cual devuelve la posición relativa entre dos drones
    #********************************************************************
    relativePosition = np.array([drone0.position.x - drone1.position.x,
                                  drone0.position.y - drone1.position.y,
                                  drone0.position.z - drone1.position.z])
    return relativePosition

def get_Resolution_Path(drone):
    #*******************************************************************
    # Función con la que se obtiene el ultimo camino seguido por un dron
    #*******************************************************************
    resolutionPath = []
    path = drone.path[::-1]
    path = path[1:]

    for point in path:

        if point != drone.startPoint:

            resolutionPath.append(point)

        else:

            resolutionPath.append(point)
            break

    resolutionPath = resolutionPath[::-1]
    return resolutionPath

def move_All_Drones(traffic, delta_t,restrictions,t):
    #*******************************************************************************************
    # Función la cual mueve a todos los drones de la lista "traffic" un cierto periodo de tiempo
    # en función de las restricciones de la lista "restrictions" 
    #*******************************************************************************************
    for drone in traffic:

        try: 
            found = False

            if restrictions[t] != None:

                for restriction in restrictions[t]:

                    if str(drone.id) == restriction:

                        found = True
                        drone.solution_waypoints.append(drone.position)
                        drone.restriction_solution.append(True)

            if found == False: 

                drone.move(delta_t)

        except: drone.move(delta_t)
            

def get_id_number(drone):
    #*******************************************************************************************
    # Función la cual devuelve el identificador de un don
    #*******************************************************************************************
    try:
        return int(drone.id[1:])  # Suponemos que el número sigue a la letra "D"
    except ValueError:
        return float('inf')

def sort_traffic_by_name(traffics):
    #*******************************************************************************************************
    # Función la cual ordena la lista "traffics" en función de los identificadores de los drones de la lista
    #*******************************************************************************************************
    return sorted(traffics, key=get_id_number)

def get_actual_time():
    #***********************************************************
    # Función la cual obtiene el tiempo actual en cierto fromato
    #***********************************************************
    actual_time = datetime.datetime.now()
    # Formatear la fecha y hora actual como una cadena
    actual_time_format = actual_time.strftime('%Y_%m_%d_%H_%M_%S')
    return actual_time_format


def full_json_solution_builder(sender,traffic, delta_t, safe_factor):
    #*******************************************************************
    # Función la cual construye un json con las soluciones del algoritmo
    #*******************************************************************
    jsonWaypoints = {}

    for drone in traffic:

        jsonWaypoints[f'{drone.id}'] = [waypoint.pretty_print_point() for waypoint in drone.solution_waypoints]

    jsonRestrictions = {}

    for drone in traffic:

        jsonRestrictions[f'{drone.id}'] = drone.restriction_solution
         
    
    json_tx = {

        'sender': sender,
        'solution_waypoints': jsonWaypoints,
        'solution_restrictions': jsonRestrictions,
        'delta_t': delta_t,
        'safe_factor': safe_factor,
        'total_traffics': len(traffic)
    }

    return json_tx

def build_solution_file(waypoint_solution_for_me, restrictions_solution_for_me, droneId, currentWaypoint,delta_t,own_position):
    #*****************************************************************************************************************
    # Función la cual construye un archivo .waypoint en el formato que espcifica MAVLink con el plan de vuelo solución
    #*****************************************************************************************************************
    print(f"currentWaypoint: {currentWaypoint}")

    with open(f'waypoints_file_solution_{droneId}.waypoints','w', encoding='utf-8') as solution_file:
        
        solution_file.write("QGC WPL 110\n")

        seq=0
        i=0

        print(f"len waypoints: {len(waypoint_solution_for_me)}")
        print(waypoint_solution_for_me)
        print(f"len restrictions: {len(restrictions_solution_for_me)}")
        print(restrictions_solution_for_me)

        while len(restrictions_solution_for_me) > i:

            waiting_time = 0

            if restrictions_solution_for_me[i] == True:

                while restrictions_solution_for_me[i+1] == True:

                    waiting_time += delta_t
                    i+=1
                    
                param1 = "{:.8f}".format(waiting_time)#Seconds to wait
                print(param1)
                command = 19
                    
            else:
                command = 82
                param1 = "{:.8f}".format(0)         
            

            waypoint = waypoint_solution_for_me[i] 
            x,y,z = waypoint[0],waypoint[1],waypoint[2]
            lat,lon,alt = geo_functions.ecef_to_lla(x,y,z)
            lat_round = "{:.8f}".format(lat)
            lon_round = "{:.8f}".format(lon)
            alt_round = "{:.8f}".format(alt)
            current = 0; frame = 0; param2 = "{:.8f}".format(0); param3 = "{:.8f}".format(0); param4 = "{:.8f}".format(0); autocontinue = 1

            if seq == int(currentWaypoint):
                current = 1 

            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (seq,current,frame,command,param1,param2,param3,param4,lat_round,lon_round,alt_round,autocontinue)
            solution_file.write(commandline)

            seq += 1; 
            i += 1

            
    file_name = f'waypoints_file_solution_{droneId}.waypoints'
    
    return file_name