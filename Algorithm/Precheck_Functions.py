#Precheck functions

import time
import Common_Functions

def route_min_distance_check(traffic):
    #************************************************************************************************************************************
    # Función que comprueba todos los segmentos de todas las rutas en busca de algún posible conflicto (distancia minima entre segmentos)
    #************************************************************************************************************************************
    i=0
    drone = traffic[i]
    nextDrone = traffic[i+1]
    while len(traffic)-1 > i:
        try:
            drone = traffic[i]
            maxDistance = max([drone.conflictRadius,nextDrone.conflictRadius])

            j=0
            if len(drone.route) >= len(nextDrone.route):
                while len(drone.route) > j:

                    for line in drone.route:

                        try:
                            if line.min_Distance_To_Other_Line(nextDrone.route[j]) <= maxDistance:
                                return True
                        except: continue
                    j += 1
            else:
                while len(nextDrone.route) > j:

                    for line in nextDrone.route:
                        
                        try:
                            if line.min_Distance_To_Other_Line(drone.route[j]) <= maxDistance:
                                return True
                        except: continue
                    j += 1
                    
            nextDrone = traffic[traffic.index(nextDrone)+1]
        except:
            i += 1

    return False

def check_parallel_routes(traffic,delta_t, stop_event):
    #***************************************************************************************************************
    # Función que calculará si dos rutas están bajo posible conflicto durante mas de un cierto porcentaje de la ruta
    #***************************************************************************************************************
    
    parallel_time = time.time()
    parallelRoutes=[]  
    i=0
    drone = traffic[i]
    nextDrone = traffic[i+1]

    while len(traffic) > i and not stop_event:

        try:
            conflictPoints = 0
            steps = 0
            drone = traffic[i]
            maxDistance = max([drone.conflictRadius,nextDrone.conflictRadius])

            while drone.status != 'ended':

                for lines in nextDrone.route:

                    distance = lines.distance_To_Point(drone.position)

                    if distance < maxDistance:

                        conflictPoints += 1

                steps += 1
                drone.move(delta_t)
            
            if conflictPoints / steps > 0.8: #Por definición del problema, dos drones no pueden iniciar en conflicto

                if drone not in parallelRoutes: 

                    parallelRoutes.append(drone)

                if nextDrone not in parallelRoutes: 

                    parallelRoutes.append(nextDrone)

            drone.reset()
            nextDrone = traffic[traffic.index(nextDrone)+1]
            
            
        except:
            i += 1
            try: nextDrone = traffic[i+1]; 
            except: 
                print("--- Parallels check: %s seconds ---" % (time.time() - parallel_time))
                Common_Functions.reset_All_Drones(traffic) 
                return parallelRoutes
 
def initial_distance_check(traffic,safe_factor):
    #**********************************************************************************************
    # Función que hace un chequeo inicial para evitar empezar la simulación con drones en conflicto
    #**********************************************************************************************
    i=0

    for i in range(len(traffic)):

        for j in range(i+1,len(traffic)):

            maxDistance = int(max([traffic[i].conflictRadius,traffic[j].conflictRadius])) + safe_factor
            distance = Common_Functions.distance_Between_Drones(traffic[i],traffic[j])

            if distance <= maxDistance:

                print(f"Initial positions too much closer:  {traffic[i].id}  &  {traffic[j].id} | Distance: {distance}")

                return False
            
            j += 1
        i += 1
    return True