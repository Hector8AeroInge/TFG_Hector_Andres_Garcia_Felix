#Traffic lights algorithm functions

import Common_Functions, classes
import numpy as np
import excel_traces_generator

def restriction_Maker(conflicts,restrictions,n):
    #***********************************************************************************************************************
    # Función encargada de generar las restricciones para cada iteración en función de los conflictos que se le den de input
    #***********************************************************************************************************************
    for conflict in conflicts:

        slaveDrone = conflict.conflictedTraffics[0] #El dron slave siempre sera el primero del la lista prio (0-5)

        if conflict.t > 0:
            try:
                list = restrictions[conflict.t-1]
                if slaveDrone.id in list == False:

                    Common_Functions.agregar_objeto_en_posicion(restrictions,slaveDrone.id,conflict.t-1)
                    
                else: Common_Functions.agregar_objeto_en_posicion(restrictions,slaveDrone.id,conflict.t-n)

            except: Common_Functions.agregar_objeto_en_posicion(restrictions,slaveDrone.id,conflict.t-1)  #Primera exec no existen restricciones
        
        else:
            Common_Functions.agregar_objeto_en_posicion(restrictions,slaveDrone.id,0)

    return restrictions



def check_For_Conflict(traffic,restrictions,parallelRoutes,delta_t, stop_event, traces_flag):
    #************************************************************************************************************************************************************
    # Función principal del algoritmo "Traffic lights". Consiste en hacer a los drones recorrer sus rutas  la par que se calculan los puntos de conflicto
    # Con estos puntos de conflicto, posterirormente se realizan mas iteración aplicando las restricciones generadas hasta llegar a una iteración de 0 conflictos
    #************************************************************************************************************************************************************

    
    if traces_flag == "1":

        excel_file, file_route = excel_traces_generator.crate_excel_file("simulation_solution")
        excel_traces_generator.add_sheet_excel_traces(excel_file,file_route,"simulation_solution")

    time=0; t=0; conflicts = []

    Common_Functions.reset_All_Paths(traffic)
    Common_Functions.reset_all_drones_solution_waypoints(traffic)
    Common_Functions.reset_all_drones_solution_restrictions(traffic)
    
    while all(agents.status == 'ended' for agents in  traffic) == False and not stop_event:
        i=0; drone = traffic[i]; nextDrone = traffic[i+1]

        for i in range(len(traffic)):

            for j in range(i+1,len(traffic)):

                drone = traffic[i]; nextDrone = traffic[j]

                distance = Common_Functions.distance_Between_Drones(drone,nextDrone)

                #Check if both drones are on route
                bothOnRoute = False
                if (drone.status == nextDrone.status == 'on route'):
                    bothOnRoute = True

                droneInParallel = 0
                nextDroneInParallel = 0
                flag = 0
                
                if drone in parallelRoutes:

                    droneInParallel = 1

                if nextDrone in parallelRoutes:

                    nextDroneInParallel = 1

                    if droneInParallel == nextDroneInParallel == 1:

                        flag = 1
                        continue
                    
                if (distance < drone.conflictRadius or distance < nextDrone.conflictRadius) and bothOnRoute == True:
                    if flag == 0:    
                        conflictedTrafics = sorted([drone,nextDrone],key=lambda x: x.priority) #Si los dos tienen la misma prioridad se ordenan random
                        if droneInParallel == 1 and nextDroneInParallel == 0:
                            conflictedTrafics = [nextDrone,drone]
                        if droneInParallel == 0 and nextDroneInParallel == 1:
                            conflictedTrafics = [drone,nextDrone]
                      
                        positions = [drone.position, nextDrone.position]
                        conflicts.append(classes.Conflict(conflictedTrafics,t,positions))   
                               
        Common_Functions.update_All_Drones_Path(traffic)

        if traces_flag == "1":
            for drone in traffic:
                excel_traces_generator.add_single_drone_trace(excel_file, file_route,"simulation_solution","FinalSolution",traffic,drone,t)

        Common_Functions.move_All_Drones(traffic,delta_t,restrictions,t)

        if all(agents.status == 'ended' for agents in traffic) == True:  #Tratar de eliminar operacion tan pesada
            if traces_flag == "1":
                return file_route
            Common_Functions.reset_All_Drones(traffic)
            #print(',,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,')
            return conflicts
                                    #Quiza se puede eliminar este if?¿?¿???
        time += delta_t
        t += 1
        #excel_Log_Add_Data(dataFrame, traffic, n, t, time)
    
    if traces_flag == "1":
        #print(',,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,')
        return file_route

    Common_Functions.reset_All_Drones(traffic)
    return conflicts
