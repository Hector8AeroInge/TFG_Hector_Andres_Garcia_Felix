#sim_init will caontain all functions to initialize the local simulation of the raw algorithm

import geo_functions
import os
import sys

# Obtiene la ruta del directorio actual (carpeta_scripts)
script_dir = os.path.dirname(os.path.abspath(__file__))
# Construye la ruta al directorio del paquete (carpeta_modulos)
paquete_dir = os.path.join(script_dir, '..', 'Algorithm')
# Añade el directorio del paquete al sys.path
sys.path.append(paquete_dir)

import classes


def decode_flight_plan(txt_file):
    #****************************************************************************************
    # Función que decodifica la información de los planes de vuelo en base al formato MAVLink
    #****************************************************************************************

    "First (format and version): QGC WPL <VERSION>"
    "Other lines: <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>"
    waypoints=[]

    with open(txt_file,'r') as file:

        for i, rows in enumerate(file):

            if i > 1:
                waypoint_split = rows.strip().split('\t')
                lat = waypoint_split[8]
                lon = waypoint_split[9]
                alt = waypoint_split[10]
                waypoint = [lat,lon,alt]
                waypoints.append(waypoint)

    return waypoints
    

def build_simulation_drones(flight_plans, own_id,own_position,own_waypoints,own_velocity,own_conflictRadius,own_priority,own_currentWaypoint):
    #************************************************************************************************
    # Función que devuelve los traficos como objetos Drone para poder inicializar la simulación local
    #************************************************************************************************
    '''
    self.id = id   <<<<
    self.position = position <<<<
    self.path = path           #Made of waypoints
    self.route = route         #Made of continuous lines
    self.waypoints = waypoints <<<<
    self.velocity = velocity <<<<
    self.startPoint = startPoint
    self.endPoint = endPoint
    self.conflictRadius = conflictRadius <<<<
    self.priority = priority <<<<
    self.status = status 
    self.heading = heading 
    self.currentSegment = currentSegment 
    self.currentWaypoint = currentWaypoint 
    self.RHO_applied = RHO_applied 
    '''  
    traffic = [build_drone(own_id,own_position,own_waypoints,own_velocity,own_conflictRadius,own_priority,own_currentWaypoint)]
    for drones_flightplan in flight_plans:
        
        "[sender,position,waypoints,velocity,conflictRadius,priority]"
        
        traffic.append(build_drone(drones_flightplan[0],drones_flightplan[1],drones_flightplan[2],drones_flightplan[3],drones_flightplan[4],drones_flightplan[5],drones_flightplan[6]))

    return traffic

def build_drone(id,position,waypoints,velocity,conflictRadius,priority,currentWaypoint):
    #*****************************************************************************************************************
    # Función que construye de manera individual los drones para la simulación. Adapta los parametros a los necesarios
    # para el correcto funcionamiento del algoritmo
    #*****************************************************************************************************************

    "[sender,position,waypoints,velocity,conflictRadius,priority]"
    
    x,y,z =  geo_functions.lla_to_ecef(position[0],position[1],position[2])
    position_converted = classes.Point(x,y,z)

    print(f"\tBuilding {id}")
    print(f"\tInitial possition ECEF: {position_converted.pretty_print_point()}")
    print(f"\tInitial waypoint: {currentWaypoint}")

    currentWaypoint = int(currentWaypoint) - 1  #Discordance between flight planner and algorithm

    waypoints_converted =[]
    for waypoint in waypoints:
        
        x,y,z = geo_functions.lla_to_ecef(waypoint[0],waypoint[1],waypoint[2])
        waypoints_converted.append(classes.Point(x,y,z))
        print(f"\t{x},{y},{z}")

    newDrone = classes.Drone(
        id,position_converted,waypoints_converted,velocity,
        conflictRadius,priority,currentWaypoint)

    return newDrone

