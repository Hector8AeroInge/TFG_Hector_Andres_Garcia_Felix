# .----------------. .----------------. .----------------. 
#| .--------------. | .--------------. | .--------------. |
#| |  _______     | | |  ____  ____  | | |     ____     | |
#| | |_   __ \    | | | |_   ||   _| | | |   .'    `.   | |
#| |   | |__) |   | | |   | |__| |   | | |  /  .--.  \  | |
#| |   |  __ /    | | |   |  __  |   | | |  | |    | |  | |
#| |  _| |  \ \_  | | |  _| |  | |_  | | |  \  `--'  /  | |
#| | |____| |___| | | | |____||____| | | |   `.____.'   | |
#| |              | | |              | | |              | |
#| '--------------' | '--------------' | '--------------' |
# '----------------' '----------------' '----------------' 

import numpy as np
import Common_Functions
import matplotlib.pyplot as plt
import time
import DrawingTools
import excel_traces_generator
import classes
import math

#Creación del archivo excel que contendrá las trazas de la ejecución RHO
excel_file, file_route = excel_traces_generator.crate_excel_file("RHO")
excel_traces_generator.add_sheet_excel_traces(excel_file,file_route,"RHO")


def compute_heading(drone, neighbor_agents, delta_t, fences, safe_factor, t, conflicts,time_horizon):
    #**********************************************************************************
    # Función la cual calculará el rumbo de cada dron según la logica del RHO
    # En función de la prioridad y la situación de cada dron el rumbo calculado
    # sera uno u otro. Para ello se tendrán en cuenta los demas drones y los obstaculos
    #**********************************************************************************
    if drone.status != 'ended':

        drone_pref_heading = drone.position.calculate_unitary_Vector(drone.waypoints[drone.currentWaypoint])
        new_heading = drone_pref_heading

        headings = []

        for agent in neighbor_agents:

            if agent != drone and agent.priority >= drone.priority and agent.status != 'ended':

                relative_position = Common_Functions.relative_Position_Between_Drones(agent,drone)
                distance_between_drones = Common_Functions.distance_Between_Drones(agent,drone)
                
                conflict_radius = max([drone.conflictRadius,agent.conflictRadius]) #Added to the route to actuate earlier than conflict is done
                conflict_radius_plus_factor = conflict_radius + safe_factor

                n_delta_t_horizon = math.ceil(time_horizon / delta_t)
                future_conflict = False

                try:

                    for conflict in conflicts:

                        if conflict.t == t + n_delta_t_horizon:

                            if drone in conflict.conflictedTraffics and agent in conflict.conflictedTraffics:

                                future_conflict = True
                                break       
                except: pass

                if distance_between_drones < conflict_radius_plus_factor or future_conflict == True:    #El radio de conflicto no deja de ser una indicador de la delicadeza de cada agente
                    #Solo tomará en consideración los agentes dentro de esta suma de radios o con los que pueda tener conflicto en el horizonte futuro
                    if distance_between_drones < conflict_radius_plus_factor:

                        if np.linalg.norm(relative_position) > 0.0001:   #Evitar divisiones entre 0

                            new_heading += relative_position / np.linalg.norm(relative_position) * (drone_pref_heading - np.linalg.norm(drone.heading))
                            prio_factor = 1 / distance_between_drones
                            anti_drone_new_heading = classes.Heading(new_heading,prio_factor)
                            headings.append(anti_drone_new_heading)
                            
                    elif distance_between_drones > conflict_radius_plus_factor or future_conflict == True:

                        if distance_between_drones > 0.0001:  #Está en la region de radios sumados pero no en la zona de potencial conflicto

                            w = max(0.0, distance_between_drones - safe_factor) / safe_factor #Aplica una corrección de heading con cierto factor de agudez (mas cerca mas abrupto el giro)
                            new_heading += w * relative_position / distance_between_drones * (drone_pref_heading - np.linalg.norm(drone.heading))
                            new_heading = new_heading /np.linalg.norm(new_heading)
                            prio_factor = 1 /  distance_between_drones
                            anti_drone_new_heading = classes.Heading(new_heading,prio_factor)
                            headings.append(anti_drone_new_heading)

                for fence in fences:

                    fence_intersect_point = fence.does_drone_intersect(drone.heading)

                    if type(fence_intersect_point) != bool:

                        distance_between_drone_fence = fence.distance_to_point(drone.get_position_as_array())

                        if distance_between_drone_fence < drone.conflictRadius:

                            relative_heading = drone.heading - fence.normal
                            wall_paralel_vector = fence.project_vector_onto_plane(drone.heading)
                            prio_factor = 1 / fence.distance_to_point(drone.get_position_as_array())
                            anti_Wall_new_heading = classes.Heading(anti_Wall_new_heading,prio_factor)
                            headings.append(wall_paralel_vector)

        if not headings:

            drone.status = 'on route'
            return new_heading
        
        else:

            drone.status = 'avoiding'
            return ponderate_heading(headings)
        
    else: return np.array([0,0,0]) #Dron quieto

        
def solve_Parallel_Routes_RHO(parallelRoutes,delta_t,walls,safe_factor,time_horizon, conflicts, stop_event):
    #*************************************************************************************************************
    # Función que moverá los drones aplicando las correciones RHO hasta que los drones lleguen a su posición final
    #*************************************************************************************************************
    solveParallelStartTime = time.time()

    print("##Running RHO algorithm")
    trafficsSorted = sorted(parallelRoutes, key=lambda x: x.priority)

    for drone in trafficsSorted:

        drone.path = []

    t=0

    # Bucle el cual recorrerá todos los drones moviendolos en funcion de las regulaciones calculadas en la iteración previa
    # hasta que todo hayan alcanzado el final de sus rutas
    while all(drone.status == 'ended' for drone in parallelRoutes) == False and not stop_event:
        
        headings = []
        for drone in trafficsSorted:
            headings.append(compute_heading(drone,trafficsSorted,delta_t,walls,safe_factor,t,conflicts,time_horizon))

        i=0
        for drone in trafficsSorted:
            drone.heading = headings[i]
            i += 1
        
        Common_Functions.move_All_Drones(trafficsSorted,delta_t,[],t) #[] Because no restriction applied
        
        for drone in trafficsSorted:
           excel_traces_generator.add_single_drone_trace(excel_file, file_route,"RHO","RHO",trafficsSorted,drone,t)
        
        Common_Functions.update_All_Drones_Path(trafficsSorted)
        t += 1

    for drone in trafficsSorted:

        drone.RHO_applied = True
        drone.set_Path_As_Route()
        drone.set_Path_As_Way_Points()
        drone.waypoints = drone.solution_waypoints

    Common_Functions.reset_All_Drones(trafficsSorted)

    print("--- Parallel_solution: %s seconds ---" % (time.time() - solveParallelStartTime))


def ponderate_heading(headings):
    #****************************************************************************************************************
    # Función que calculará el heading resultante en base a sus propios factores de la lista de objetos tipo Heading.
    #***************************************************************************************************************

    total_sum = sum(heading.prio_factor for heading in headings)
    normalized_factors = [heading.prio_factor/total_sum for heading in headings]

    result = np.zeros_like(headings[0].direction)

    for i in range(len(headings)):
        
        result += normalized_factors[i] * headings[i].direction 
    
    result_normalized = result / np.linalg.norm(result)
    
    return result_normalized
