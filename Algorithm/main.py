# _____ _____ ____ 
#|_   _|  ___/ ___|
#  | | | |_ | |  _ 
#  | | |  _|| |_| |
#  |_| |_|   \____|
                  
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
from mpl_toolkits.mplot3d import Axes3D
import time
import sys
import datetime
import pandas as pd
import excel_traces_generator
import threading
from matplotlib.figure import Figure
import os


#*************************Import py files*************************
import classes, Common_Functions, DrawingTools, Precheck_Functions, Secondary_Functions, Traffic_Lights_Functions, RHO_Functions


#********************************************************************
# Iniciador de archivo excel para guardar las trazas de la simulación
#********************************************************************
#excel_file, file_route = excel_traces_generator.crate_excel_file("General")
#excel_traces_generator.add_sheet_excel_traces(excel_file,file_route,"General")


def simulate(traffics,walls,delta_t,safe_factor,log_traces_flag, nominal_functioning_flag,stop_event,droneId):
    #*******************************************************************************************************************
    # Función principal del algoritmo. En esta función se ejecutan la secuencia de funciones que componen el algortimo
    # Esta será la función que ejecutarán los drones de manera local por tal de obtener una solución libre de conflictos
    #*******************************************************************************************************************

    simulationStartTime = time.time()
    trafficsSorted = sorted(traffics, key=lambda x: x.priority)

    if Precheck_Functions.initial_distance_check(traffics,safe_factor) == False and not stop_event.is_set():

        print('Not possible to simulate. At least two drones are beggining in conflict positions')

        if nominal_functioning_flag == '0':

            plot_routes0_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes0_thread.start()

        return -1,-1
    
    if Precheck_Functions.route_min_distance_check(traffics) == True and not stop_event.is_set():
        #************Pre solutions check************

        print('****Traffic sorted by priority****')
        conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, [], [],delta_t,stop_event.is_set(),"0"); 
        parallelRoutes = Precheck_Functions.check_parallel_routes(trafficsSorted, delta_t, stop_event.is_set())

        Common_Functions.reset_all_drones_solution_waypoints(trafficsSorted)
        Common_Functions.reset_all_drones_solution_restrictions(trafficsSorted)

        print('****Parallel routes identified****')
        print(f"Total parallel routes identified: {len(parallelRoutes)}")

        #************RHO solution for parallel routes************

        if len(parallelRoutes) > 0:

            RHO_Definitive.solve_Parallel_Routes_RHO(parallelRoutes, delta_t,walls,safe_factor,conflicts,stop_event.is_set())

        if nominal_functioning_flag == '0':

            plot_routes1_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes1_thread.start()


        print('****Parallel routes solved****')
        #************Traffic lights solution calculator**********

        conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, [], parallelRoutes,delta_t,stop_event.is_set(),"0"); print('conflicts_t0: ',len(conflicts))  # [] Because no restriction applied
        restrictions = []
        n=0

        while len(conflicts) != 0:

            print('n: ' , n)
            restrictions = Traffic_Lights_Functions.restriction_Maker(conflicts, restrictions, n)
            conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, restrictions, parallelRoutes, delta_t,stop_event.is_set(),"0")
            print('conflicts: ',len(conflicts))

            n += 1

        if nominal_functioning_flag == '0':

            plot_routes2_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes2_thread.start()

        Common_Functions.set_path_as_waypoint_All_Drones(trafficsSorted)

        #Iguala el tamaño de las soluciones
        if len(traffics[0].waypoints) > len(restrictions):

            diff = len(traffics[0].waypoints) - len(restrictions)
            restrictions.extend([None]*diff)


        print('****Conflicts solved****')

        if not stop_event.is_set():

            waypoints_solution_and_restrictions = Common_Functions.full_json_solution_builder(droneId,trafficsSorted,delta_t,safe_factor)

            print("--- Simulation: %s seconds ---" % (time.time() - simulationStartTime))

            return waypoints_solution_and_restrictions
        
        else:
            print("Algorithm interrupted due to new traffic")
            return -2,-2

    else:
        print("No regulation needed")
        
        if nominal_functioning_flag == '0':
            
            DrawingTools.plot_Routes(trafficsSorted,walls)
        
        return 0,0


def simulate_panel(traffics,walls,delta_t,safe_factor,log_traces_flag, nominal_functioning_flag,stop_event):
    #*****************************************************************************************************************
    # Función igual que la principal del algortimo pero con modificaciones para brindar al panel de la interfaz gráfica
    # los datos necesarios para poder mostrar en pantalla las trazas y la animación del calculo del algoritmo
    #*****************************************************************************************************************

    simulationStartTime = time.time()
    trafficsSorted = sorted(traffics, key=lambda x: x.priority)

    if Precheck_Functions.initial_distance_check(traffics,safe_factor) == False and not stop_event.is_set():

        print('Not possible to simulate. At least two drones are beggining in conflict positions')

        if nominal_functioning_flag == '0':

            plot_routes0_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes0_thread.start()

        return -1,-1
    
    if Precheck_Functions.route_min_distance_check(traffics) == True and not stop_event.is_set():

        #************Pre solutions check************

        conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, [], [],delta_t,stop_event.is_set(),"0"); 
        parallelRoutes = Precheck_Functions.check_parallel_routes(trafficsSorted, delta_t, stop_event.is_set())

        Common_Functions.reset_all_drones_solution_waypoints(trafficsSorted)
        Common_Functions.reset_all_drones_solution_restrictions(trafficsSorted)

        #************RHO solution for parallel routes************

        if len(parallelRoutes) > 0:

            RHO_Definitive.solve_Parallel_Routes_RHO(parallelRoutes, delta_t,walls,safe_factor,conflicts,stop_event.is_set())

        if nominal_functioning_flag == '0':

            plot_routes1_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes1_thread.start()

        #************Traffic lights solution calculator**********
            
        conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, [], parallelRoutes,delta_t,stop_event.is_set(),"0"); print('conflicts_t0: ',len(conflicts))  # [] Because no restriction applied
        restrictions = []
        n=0

        while len(conflicts) != 0:

            restrictions = Traffic_Lights_Functions.restriction_Maker(conflicts, restrictions, n)
            conflicts = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, restrictions, parallelRoutes, delta_t,stop_event.is_set(),"0")

            n += 1

        traces_file_path = Traffic_Lights_Functions.check_For_Conflict(trafficsSorted, restrictions, parallelRoutes, delta_t,stop_event.is_set(),"1")

        if nominal_functioning_flag == '0':

            plot_routes2_thread = threading.Thread(target=DrawingTools.plot_Routes, args=(traffics,walls))
            plot_routes2_thread.start()

        Common_Functions.set_path_as_waypoint_All_Drones(trafficsSorted)

        #Iguala el tamaño de las soluciones
        if len(traffics[0].waypoints) > len(restrictions):
            diff = len(traffics[0].waypoints) - len(restrictions)
            restrictions.extend([None]*diff)

        if not stop_event.is_set():

            print("--- Simulation: %s seconds ---" % (time.time() - simulationStartTime))

            return traces_file_path, trafficsSorted
           
        else:
            print("Algorithm interrupted due to new traffic")

            return -2,-2

    else:
        print("No regulation needed")
        
        if nominal_functioning_flag == '0':
            
            DrawingTools.plot_Routes(trafficsSorted,walls)
        
        return -1,-1






'''
#%% *********************Init*********************
traffics = [Drone_Declaration.Drone_0,Drone_Declaration.Drone_1, Drone_Declaration.Drone_2, Drone_Declaration.Drone_3]
walls = [Drone_Declaration.wall_1]
delta_t = 0.25 
safe_factor = 2
traffics = simulate(traffics,walls,delta_t,safe_factor)



#%% *********************Grafica de rutas*********************
resolutionPaths = []
for drone in traffics:
    resolutionPaths.append(Common_Functions.get_Resolution_Path(drone))
max_length = max(len(resolution) for resolution in resolutionPaths)
for i in range(len(resolutionPaths)):
        last_value = resolutionPaths[i][-1] if resolutionPaths[i] else None  # Último valor de la lista
        resolutionPaths[i].extend([last_value] * (max_length - len(resolutionPaths[i])))



#%% *********************Animación*********************
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
puntos = [ax.scatter([], [], [], marker='o') for _ in range(len(traffics))]

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

num_points = len(max(resolutionPaths, key=len))


point1, = ax.plot([], [], [], 'o', markersize=5, label='D0')
point2, = ax.plot([], [], [], 'o', markersize=5, label='D1')
point3, = ax.plot([], [], [], 'o', markersize=5, label='D2')
point4, = ax.plot([], [], [], 'o', markersize=5, label='D3')


#vertices = [wall_1.A, wall_1.B, wall_1.C, wall_1.D]
#faces = [[vertices[0], vertices[1], vertices[2]], [vertices[1], vertices[2], vertices[3]]]
         
#ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

def init():
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 30)
    ax.set_zlim(0, 15)
    ax.set_title('4 Drones Sim')
    ax.legend()
    for drones in traffics:
        for lines in drones.route:
            DrawingTools.add_line_to_figure(ax,lines,drones.id)

    return point1, point2, point3, point4

def update(frame):
    # Obtener las coordenadas x, y, z para el cuadro actual
    x1 = resolutionPaths[0][frame].x
    y1 = resolutionPaths[0][frame].y
    z1 = resolutionPaths[0][frame].z
    
    x2 = resolutionPaths[1][frame].x
    y2 = resolutionPaths[1][frame].y
    z2 = resolutionPaths[1][frame].z

    
    x3 = resolutionPaths[2][frame].x
    y3 = resolutionPaths[2][frame].y
    z3 = resolutionPaths[2][frame].z
    
    x4 = resolutionPaths[3][frame].x
    y4 = resolutionPaths[3][frame].y
    z4 = resolutionPaths[3][frame].z
    
    # Actualizar los datos de los puntos
    point1.set_data(x1, y1)
    point1.set_3d_properties(z1)
    
    point2.set_data(x2, y2)
    point2.set_3d_properties(z2)
    
    point3.set_data(x3, y3)     
    point3.set_3d_properties(z3)
    
    point4.set_data(x4, y4)
    point4.set_3d_properties(z4)
    
    return point1, point2, point3, point4

ani = FuncAnimation(fig, update, frames=num_points, init_func=init, blit=True, repeat=True, interval=80)
#ani.save('test_case_0.gif', writer='pillow')
# Mostrar la animación
plt.show()
'''