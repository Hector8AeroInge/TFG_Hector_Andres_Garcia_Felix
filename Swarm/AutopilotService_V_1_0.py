import json
import math
import threading
import paho.mqtt.client as mqtt
import time
import dronekit
from dronekit import connect, Command, VehicleMode
from paho.mqtt.client import ssl
from pymavlink import mavutil
from geo_functions import geopy_distance
import network_functions
import sim_init
from mission_import_export import upload_mission
import os
import sys
from pprint import pprint

# Obtiene la ruta del directorio actual (carpeta_scripts)
script_dir = os.path.dirname(os.path.abspath(__file__))
# Construye la ruta al directorio del paquete (carpeta_modulos)
paquete_dir = os.path.join(script_dir, '..', 'Algorithm')
# Añade el directorio del paquete al sys.path
sys.path.append(paquete_dir)

import main
import Common_Functions



def get_telemetry_info ():
    #******************************************************************
    # Función que devuelve el json con los datos de telemetria del dron
    #******************************************************************

    global state
    global currentWaypoint
    if 'currentWaypoint' not in globals():
        currentWaypoint = "0"
    telemetry_info = {
        'id':  droneId,
        'lat': vehicle.location.global_frame.lat,
        'lon': vehicle.location.global_frame.lon,
        'heading': vehicle.heading,
        'groundSpeed': vehicle.groundspeed,
        'altitude': vehicle.location.global_relative_frame.alt,
        'battery': vehicle.battery.level,
        'state': state,
        'NW_state': current_network_status,
        'NW_subNet': current_subnetwork,
        'priority': priority,
        'currentWP': currentWaypoint
    }
    return telemetry_info


def send_telemetry_info():
    #*******************************************************
    # Función que publica los datos de telemetria en el MQTT
    #*******************************************************
    global external_client
    global sending_telemetry_info
    global sending_topic

    while sending_telemetry_info:

        time.sleep(1.5)
        external_client.publish(droneId + "/traffic/telemetryInfo", json.dumps(get_telemetry_info()))
        internal_client.publish(droneId + "/traffic/telemetryInfo", json.dumps(get_telemetry_info()))


def distanceInMeters(aLocation1, aLocation2):
    #*************************************************************************
    # Función que calcula la distancia entre dos puntos de la esfera terrestre
    #*************************************************************************
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def newtraffic_function(message):
    #******************************************************************************************************************
    # Función encargada de gestinoar el mensaje "newTraffic". Cuando se recibe el mensaje, esta función publica el plan 
    # de vuelo del dron en su correspondiente subNet
    #******************************************************************************************************************
    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork
    global currentWaypoint
    global all_flight_plans
    global solution_confirmations


    payload_json = json.loads(message.payload.decode())
    currentWaypoint = str(vehicle.commands.next)

    all_flight_plans = []; solution_confirmations=[] #Restart all the cicle by removing all drones registered
    print(f'\t@all_flight_plans: {len(all_flight_plans)}')
    print(f"\tNew traffic entered the zone: {payload_json.get('sender')} | {droneId} publishing flight plan info\n")

    flightPlan = {'sender': droneId, 
                    'position': [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_relative_frame.alt],
                    'waypoints': waypoints_file,
                    'velocity': velocity,
                    'conflictRadius': conflictRadius,
                    'priority': priority,
                    'currentWaypoint': currentWaypoint
                    }

    external_client.publish(f"{droneId}/{current_subnetwork}/flightPlan_info", json.dumps(flightPlan))
    print(f'Flight plan from {droneId} published\n')


def Solution_confirmation_function(message):
    #********************************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "Solution_confirmation". Si el número de veces que se ha recibido este mensaje
    # es igual al numero de tráficos en la zona, la función carga la solucion al dron y lo pone en modo "AUTO" para que ejecute la nueva solución
    #********************************************************************************************************************************************
    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork
    global currentWaypoint
    global waypoint_solution_for_me
    global solution_confirmations
    global delta_t
    global own_position
    global velocity

    if message.payload.decode() != droneId:

        solution_confirmations.append(message.payload.decode())

        if len(solution_confirmations) == len(current_in_sight_traffic):
            
            solution_file_name = Common_Functions.build_solution_file(waypoint_solution_for_me,restrictions_solution_for_me, droneId, currentWaypoint, delta_t, own_position)
            print("****************PREPARING EXECUTION OF SOLUTION****************\n")
            print(f'\tUploading solution to {droneId}...\n')
            upload_mission(solution_file_name,vehicle)

            print(f'\tSolution uploaded to {droneId}...\nExecuting solution')

            #Set cruise velocity
            vehicle.parameters['WPNAV_SPEED'] = int(velocity) * 100 #cm/s 

            #Set vehicle to auto mode
            vehicle.mode = VehicleMode("AUTO")
            print(f"\tChanging {droneId} to AUTO mode\n")
            while vehicle.mode.name != "AUTO":
                time.sleep(0.5)
            print(f"\t{droneId} changed to AUTO mode\n")

            #Ensure aircraft is armed and flying if not takeoff
            if not vehicle.armed:
                vehicle.armed = True

            while not vehicle.armed:
                print(f"\tArming {droneId}\n")
                time.sleep(0.5)
            #execute flightplan solution


def DCASsolution_function(message):
    #****************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "DCASsolution". Cuando se recibe el mensaje, la función comprueba que la solución
    # recibida es compatible con la solución calculada localmente
    #****************************************************************************************************************************

    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork
    global waypoint_solution_for_me
    global restrictions_solution_for_me
    global current_subnetwork

    print("****************ANALYZING SWARM SOLUTION****************\n")

    payload_json = json.loads(message.payload.decode())
    sender = payload_json.get('sender')
    total_traffics = payload_json.get('total_traffics')

    if sender != droneId and int(total_traffics) == (len(current_in_sight_traffic)+1):  #Descarta soluciones que sean suyas y que sean de ejecuciones pasadas
        
        print('\tDCASsolution received. Checking solution matching...\n')
        
        waypoint_solution_rx = payload_json.get('solution_waypoints')
        restrictions_solution_rx = payload_json.get('solution_restrictions')

        #Dentro de waypoints encuentra su solución:
        waypoint_solution_for_me_rx = waypoint_solution_rx.get(f"{droneId}")

        if waypoint_solution_for_me_rx is not None: #Para evitar soluciones de algoritmos en el que no he participado
            waypoint_solution_for_me_rx = [list(map(float, waypoint.split(','))) for waypoint in waypoint_solution_for_me_rx]
        
            
            restrictions_solution_for_me_rx = restrictions_solution_rx
            restrictions_solution_for_me_rx = restrictions_solution_for_me_rx.get(f"{droneId}")


            if str(waypoint_solution_for_me_rx) != str(waypoint_solution_for_me) or restrictions_solution_for_me_rx != restrictions_solution_for_me:
                #Inacabado
                print(f"Solution received from {sender}: {repr(waypoint_solution_for_me_rx)}")
                print(f"My solution: {repr(waypoint_solution_for_me)}")
                #Repetir proceso?
                json_tx = {
                    "sender":droneId
                }
                #recalcular otra vez mientras se ejecuta uno recibido
                print("\tsolutions not match!")
                external_client.publish(f"{droneId}/{current_subnetwork}/Solution_confirmation",f"{droneId}")
                
                
            else:
                print("\tSolution received and checked\n\tSending confirmation\n")
                external_client.publish(f"{droneId}/{current_subnetwork}/Solution_confirmation",f"{droneId}")
                pass


def simulate_function():
    #*************************************************************************************************************
    # Función encargada de correr la simulación de manera local. Una vez se han recibido todos los planes de vuelo
    # esta función calcula la solución en base a estos
    #*************************************************************************************************************
    global all_flight_plans
    global currentWaypoint
    global principal_process_interruption
    global waypoint_solution_for_me
    global restrictions_solution_for_me
    global delta_t
    global own_position
    global delta_t
    global safe_factor
    global time_horizon

    print("****************INITIALIZING SIMULATION****************\n")

    #Algorithm trigger
    own_position = [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_relative_frame.alt]

    traffic = sim_init.build_simulation_drones(all_flight_plans,droneId,own_position,sim_init.decode_flight_plan(f"{droneId}_waypoints.txt"),
                                                velocity,conflictRadius,priority,currentWaypoint)
    print(f"\tRunning algorithm with: ")
    pprint([drone.id for drone in traffic],compact=True)

    delta_t = int(delta_t)
    safe_factor = int(safe_factor)

    ids = [drone.id for drone in traffic]
    parameters = [delta_t,safe_factor]
    panel_trigger = {
        "sender": droneId,
        "drones": ids,
        "parameters": parameters,
    }

    external_client.publish(f"{droneId}/panel/simulationStarted", json.dumps(panel_trigger))

    json_solution = main.simulate(traffic,[],delta_t,safe_factor,time_horizon,'1',principal_process_interruption,droneId)


    if (json_solution) != 0 != -1 != -2 :
        
        waypoint_solution_for_me = json_solution.get('solution_waypoints',{})
        waypoint_solution_for_me = waypoint_solution_for_me.get(f"{droneId}")
        waypoint_solution_for_me = [list(map(float, waypoint.split(','))) for waypoint in waypoint_solution_for_me]
        
        restrictions_solution_for_me = json_solution.get('solution_restrictions')
        restrictions_solution_for_me = restrictions_solution_for_me.get(f"{droneId}")


        external_client.publish(f"{droneId}/{current_subnetwork}/DCASsolution", json.dumps(json_solution))

    else:
        print("Check the initial separation between the drones and the security distance selected")

simulation_thread = threading.Thread(target=simulate_function)

def flightPlan_info_function(message):

    #**********************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "flight_Plan_info". Cuando se recibe el mensaje, la función decodifica el plan de vuelo
    # recibido y lo almacena localmente para posteriormente usarlo en el cálculo de la solución
    #**********************************************************************************************************************************

    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork
    global principal_process_interruption
    global all_flight_plans
    global simulation_thread

    payload_json = json.loads(message.payload.decode())
    sender = payload_json.get('sender')

    if sender == droneId:
        pass
    
    else: 
        print("### RECEIVING FLIGHT PLAN ###")
        print(f'\tFlight plan received from {sender}\n')
        waypoints_fp = payload_json.get('waypoints')
        #print(waypoints_fp)

        position_fp = payload_json.get('position')
        print(f'\tposition: {position_fp}\n')

        velocity_fp = payload_json.get('velocity')
        print(f'\tvelocity: {velocity_fp}\n')

        conflictRadius_fp = payload_json.get('conflictRadius')
        print(f'\tconflictRadius: {conflictRadius_fp}\n')

        priority_fp = payload_json.get('priority')
        print(f'\tpriority: {priority_fp}\n')

        currentWaypoint_fp = payload_json.get('currentWaypoint')
        print(f'\tcurrentWaypoint: {currentWaypoint_fp}\n')

        #TEST
        with open(f'waypoints_decode_{droneId}_of_{sender}.txt','w', encoding='utf-8') as txt_file:

            print('\tDecoding received flight plan\n')
            txt_file.write(waypoints_fp) 

        waypoints_fp = sim_init.decode_flight_plan(f"waypoints_decode_{droneId}_of_{sender}.txt") #Faltaria tambien saber la velocidad de crucero de cada uno
        flight_plan_decoded = [sender,position_fp,waypoints_fp,velocity_fp,conflictRadius_fp,priority_fp,currentWaypoint_fp] 
        print(f'\t@all_flight_plans: {len(all_flight_plans)}')
        all_flight_plans.append(flight_plan_decoded)
        
        print(f'\t@current_in_sight_traffic: {len(current_in_sight_traffic)}')
        print(f'\t@all_flight_plans: {len(all_flight_plans)}')

        if len(all_flight_plans) == len(current_in_sight_traffic) != 0:
            
            if not simulation_thread.is_alive():
                simulation_thread = threading.Thread(target=simulate_function)
                simulation_thread.start()

    #flightPlan_info_thread.release()

def subNet_transfer_request_function(message):
    #**********************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "subNet_transfer_request". Cuando se recibe el mensaje, la función hace cambiar al dron
    # de subNet a la espceificada por el mensaje
    #**********************************************************************************************************************************

    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork

    payload_json = json.loads(message.payload.decode())
    subNet_topic = payload_json.get('subNet')
    requestor = payload_json.get('requestor')
    print(f"\t{requestor} Requested to join to {subNet_topic}\n")

    external_client.unsubscribe(f"+/{current_subnetwork}/#")
    print(f"\t{droneId} unsubscribed from {current_subnetwork}\n")

    external_client.subscribe(f"+/{subNet_topic}/#")
    current_subnetwork = subNet_topic

    print(f"\tTransfering all subNet to {current_subnetwork}\n")
    print(f"\t{droneId} subscribed to {subNet_topic}\n")
    sender = {'sender': droneId}
    external_client.publish(f"{droneId}/{subNet_topic}/newTraffic", json.dumps(sender))


def subNet_connection_request_function(message):
    #**************************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "subNet_connection_request". Cuando se recibe el mensaje, la función hace cambiar al dron
    # a la subNet especificada. Si el dron se encuentra conectado a otros drones, este les envia la petición a los de más de también unirse
    # a la nueva subNet
    #**************************************************************************************************************************************

    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork

    payload_json = json.loads(message.payload.decode())
    subNet_topic = payload_json.get('subNet')
    requested = payload_json.get('requested')
    sender = payload_json.get('requestor')

    if payload_json.get('requested') == droneId:

        print(f"\tReading request from {sender} to {droneId} to join {subNet_topic}\n")

        if current_network_status == 'busy' and subNet_topic != current_subnetwork:
            
            external_client.unsubscribe(f"+/{current_subnetwork}/#")
            print(f"\t{droneId} unsubscribed from {current_subnetwork}\n")

            request = {
                'requested': "All",
                'subNet': subNet_topic,
                'requestor': droneId
            }

            external_client.publish(f"{droneId}/traffic/subNet_transfer_request", json.dumps(request))
            print(f"\t{droneId} requested to join to {subNet_topic} to all drones in {current_subnetwork}\n")

            external_client.subscribe(f"+/{subNet_topic}/#")
            current_subnetwork = subNet_topic
            current_network_status = 'busy'
            print(f"\t{droneId} subscribed to {subNet_topic}\n")
        
        elif current_network_status == 'free':

            external_client.subscribe(f"+/{subNet_topic}/#")
            current_subnetwork = subNet_topic
            current_network_status = 'busy'

            print(f"\t{droneId} subscribed to {subNet_topic}\n")
            sender = {'sender': droneId}
            external_client.publish(f"{droneId}/{subNet_topic}/newTraffic", json.dumps(sender))
    else:
        current_requested_subnet.append(requested)


def telemetryInfo_function(message):
    
    #**********************************************************************************************************************************
    # Función encargada de gestinoar el mensaje "telemetryInfo". Cuando se recibe el mensaje, en base a la prioridad del dron que envia
    # el mensaje que se está recibiendo y la propia, se forman los enlaces de las subNets. Si los drones e alejan demasiado la función
    # se encarga de separarlos de las subNets
    #**********************************************************************************************************************************
    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork
    global current_requested_subnet
    global link_Distance

    link_Distance = int(link_Distance)

    #telemetryInfo_thread.join()

    payload_json = json.loads(message.payload.decode())
    sender_id = payload_json.get('id') 
            
    print('Telemetry: ' + message.payload.decode())
    print(f'Receiving telemetry from {sender_id}' )
    
    own_position = (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon)
    other_position = (payload_json.get('lat'),payload_json.get('lon'))

    if sender_id != droneId:
        print(f"distance: {geopy_distance(own_position,other_position)}")
        if sender_id not in current_in_sight_traffic:
            
            if geopy_distance(own_position,other_position) < link_Distance:
                current_in_sight_traffic.append(sender_id)
                print(f"\t{sender_id} added to current traffic\n")

                if current_network_status == payload_json.get('NW_state') == 'free':

                    if int(priority) < int(payload_json.get('priority')) and sender_id not in current_requested_subnet:
                        subNet_topic = network_functions.generate_sub_network(external_client)
                        request = {
                                'requested': sender_id,
                                'subNet': subNet_topic,
                                'requestor': droneId
                            }
                        external_client.publish(f"{droneId}/traffic/subNet_connection_request", json.dumps(request))
                        current_subnetwork = subNet_topic; current_network_status = 'busy'
                        print(f"\tcurrent_subnetwork: {current_subnetwork}; current_network_status: {current_network_status}\n")
                        external_client.subscribe(f"+/{subNet_topic}/#")
                        print(f"\t{droneId} subscribed to new subNet {subNet_topic}\n")
                        print(f"\tRequested {sender_id} to join {subNet_topic}\n")

                    elif int(priority) > int(payload_json.get('priority')):

                        pass #El otro aplicará la primera condición
                        
                    elif int(priority) == int(payload_json.get('priority')):

                        if network_functions.custom_priority(droneId,sender_id) == -1 and sender_id not in current_requested_subnet:

                            subNet_topic = network_functions.generate_sub_network(external_client)
                            request = {
                                'requested': sender_id,
                                'subNet': subNet_topic,
                                'requestor': droneId
                            }
                            external_client.publish(f"{droneId}/traffic/subNet_connection_request", json.dumps(request))
                            current_subnetwork = subNet_topic; current_network_status = 'busy'
                            external_client.subscribe(f"+/{subNet_topic}/#")
                            print(f"\t{droneId} subscribed to new subNet {subNet_topic}\n")
                            print(f"\tRequested {sender_id} to join {subNet_topic}\n")

                elif current_network_status == 'busy' and payload_json.get('NW_state') == 'free' and sender_id not in current_requested_subnet:
                    #el libre se connecta a la sub red existente
                    request = {
                                'requested': sender_id,
                                'subNet': current_subnetwork,
                                'requestor': droneId
                            }
                    external_client.publish(f"{droneId}/traffic/subNet_connection_request", json.dumps(request))
                    print(f"\t{droneId} requested {sender_id} to join {current_subnetwork}\n")

                elif current_network_status == payload_json.get('NW_state') == 'busy' and sender_id not in current_requested_subnet:

                    if payload_json.get('NW_subNet') != current_subnetwork:

                        if int(priority) > int(payload_json.get('priority')):

                            subNet_topic = network_functions.generate_sub_network(external_client)
                            request = {
                                    'requested': sender_id,
                                    'subNet': subNet_topic,
                                    'requestor': droneId
                                }
                            external_client.publish(f"{droneId}/traffic/subNet_connection_request", json.dumps(request))
                            print(f"\t{droneId} requested {sender_id} to join {subNet_topic}\n")

                        elif int(priority) < int(payload_json.get('priority')):

                            pass #El otro aplicará la primera condición

                        elif int(priority) == int(payload_json.get('priority')):
                            
                            if network_functions.custom_priority(droneId,sender_id) == -1 and sender_id not in current_requested_subnet:

                                subNet_topic = network_functions.generate_sub_network(external_client)
                                request = {
                                    'requested': sender_id,
                                    'subNet': subNet_topic,
                                    'requestor': droneId
                                }
                                external_client.publish(f"{droneId}/traffic/subNet_connection_request", json.dumps(request))
                                print(f"\t{droneId} requested {sender_id} to join {subNet_topic}\n")
                    else:
                        pass
                    # la subred de menos nodos se conecta a la red de mas nodos
        else:
            if geopy_distance(own_position,other_position) < link_Distance:

                if current_network_status == payload_json.get('NW_state') == 'busy':
                    #print(f" Suya: {payload_json.get('NW_subNet')}   mia: {current_subnetwork}")
                    if payload_json.get('NW_subNet') == current_subnetwork:
                        pass

            else:

                current_in_sight_traffic.remove(sender_id)

                if sender_id in current_requested_subnet:
                    
                    current_requested_subnet.remove(sender_id)

                print(f"###{sender_id} exited posible conflict radius")
                #Los drones se han alejado por lo tanto ya no estarn considerados en la misma zona de trafico                   
                if len(current_in_sight_traffic) == 0:
                    
                    current_not_sight_traffic = []
                    print(f"##No traffics in sight. {droneId} Exiting subNet {current_subnetwork}")
                    external_client.unsubscribe(f"+/{current_subnetwork}/#")
                    current_network_status = "free"; current_subnetwork = "none"
                    

                elif len(current_in_sight_traffic) > 1:

                    current_not_sight_traffic.append(sender_id) #se tienen en cuenta para lanzar el algoritmo
                    print(f"Added {sender_id} to not in sight traffic")

    #telemetryInfo_thread.release()

def process_message(message, client):
    
    #*********************************************************************************************************************************
    # Función principal encargada de gestionar todos los mensajes que se reciben por el servidor MQTT. En función del mensaje que se
    # reciba, se eejcutara la función pertinente de las previamente vistas. En esta función es donde se sucede el control de procesos.
    # Se inicializan hilos de procesado de manera que no se solapen procesos incompatibles y se pierdan valores de variables globales
    #*********************************************************************************************************************************

    global telemetryInfo_thread
    global subNet_connection_request_thread
    global subNet_transfer_request_thread
    global flightPlan_info_thread
    global DCASsolution_thread
    global Solution_confirmation_thread
    global newTraffic_thread
    global solution_confirmations
    global vehicle
    global sending_telemetry_info
    global sending_topic
    global op_mode
    global connection_mode
    global sending_topic
    global state
    global current_network_status; global current_subnetwork
    global waypoints_file
    global all_flight_plans
    global waypoints_solution; global restrictions_solution
    waypoints_solution = {}; restrictions_solution = []
    global solution_confirmations; 
    global waypoint_solution_for_me
    global flightPlan_info_thread
    global simulation_thread
    global waypoints_file_path


    splited = message.topic.split("/")
    origin = splited[0]
    command = splited[2]
    sending_topic = "autopilotService_" + droneId + "/" + origin
    print (f'Receiving command:  {command} *****************')

    if command == "position":
        print("Position: ", message.payload )

    if command == "connect":
        if state == 'disconnected':
            print("Autopilot service connected by " + origin)
            #para conectar este autopilotService al dron al mismo tiempo que conectamos el Mission Planner
            # hay que ejecutar el siguiente comando desde PowerShell desde  C:\Users\USER>
            #mavproxy - -master =COM12 - -out = udp:127.0.0.1: 14550 - -out = udp:127.0.0.1: 14551
            # ahora el servicio puede conectarse por udp a cualquira de los dos puertos 14550 o 14551 y Mission Planner
            # al otro

            if op_mode == 'simulation':
                if connection_mode == "global":
                    connection_string = f"tcp:127.0.0.1:{simulationPort}"
                    print("Connection to global simulated drone port: " + simulationPort)
                elif connection_mode == "local":
                    connection_string = f"tcp:127.0.0.1:{simulationPort}"
                    print("Connection to local simulated drone port: " + simulationPort)
                #connection_string = "com7"
            else:
                # connection_string = "/dev/ttyS0"
                connection_string = "com7"
                #connection_string = "udp:127.0.0.1:14550"

            vehicle = connect(connection_string, wait_ready=False, baud=115200)
            vehicle.wait_ready(True, timeout=10000)

            print ('Connected to flight controller')
            state = 'connected'

            waypoints_file = ""

            if log_Flight_Plan_Flag == "1":

                upload_mission(waypoints_file_path,vehicle)

                with open(waypoints_file_path, 'r') as file:
                    waypoints_file = file.read()
                
                with open(f"{droneId}_waypoints.txt", 'w', encoding='utf-8') as txt_file:
                    txt_file.write(waypoints_file)

                print(f"Loggin flight plan to {droneId}")
            
            vehicle.groundspeed = 1

            sending_telemetry_info = True
            y = threading.Thread(target=send_telemetry_info)
            print(f"Sending telemetry from {droneId})")
            y.start()

        else:
            print ('Autopilot already connected')

    if command == "clients":
        print(f"Conected clients: {message.payload.decode()}")
        connected_clients = message.payload.decode().split(", ")
        for client_info in connected_clients:
            client_name = client_info.split(" ")[0]
            print(f"- {client_name}")

    if command == "disconnect":
        vehicle.close()
        sending_telemetry_info = False
        state = 'disconnected'


    if command == "telemetryInfo":
        
        telemetryInfo_thread = threading.Thread(target=telemetryInfo_function,args=(message,))
        try:
            subNet_connection_request_thread.join() 
        except:
            pass

        telemetryInfo_thread.start()
                
    if command == 'subNet_connection_request':

        telemetryInfo_thread.join()
        subNet_connection_request_thread = threading.Thread(target=subNet_connection_request_function,args=(message,))
        subNet_connection_request_thread.start()

    if command == 'subNet_transfer_request':
        
        telemetryInfo_thread.join()
        subNet_transfer_request_thread = threading.Thread(target=subNet_transfer_request_function,args=(message,))
        subNet_transfer_request_thread.start()

    if command == 'flightPlan_info':
        
        #newTraffic_thread.join()
        try:
            flightPlan_info_thread.join()
            flightPlan_info_thread = threading.Thread(target=flightPlan_info_function,args=(message,))
            flightPlan_info_thread.start()
        except:
            flightPlan_info_thread = threading.Thread(target=flightPlan_info_function,args=(message,))
            flightPlan_info_thread.start()


    if command == "DCASsolution":
        try:
            simulation_thread.join()
            DCASsolution_thread = threading.Thread(target=DCASsolution_function,args=(message,))
            DCASsolution_thread.start()
        except:
            DCASsolution_thread = threading.Thread(target=DCASsolution_function,args=(message,))
            DCASsolution_thread.start()

    if command == "Solution_confirmation":
        
        DCASsolution_thread.join()
        Solution_confirmation_thread = threading.Thread(target=Solution_confirmation_function, args=(message,))
        Solution_confirmation_thread.start()

    if command == "newTraffic":

        global principal_process_interruption
        principal_process_interruption = threading.Event()

        #if simulation_thread.is_alive():

        principal_process_interruption.set()
        print("##Aborting algorithm execution, another traffic entered the zone")
        newTraffic_thread = threading.Thread(target=newtraffic_function, args=(message,))
        newTraffic_thread.start()
        principal_process_interruption.clear()

    


def on_connect(external_client, userdata, flags, rc):
    if rc==0:
        print("Connection OK")
    else:
        print("Bad connection")

def on_internal_message(client, userdata, message):
    process_message(message, internal_client)


def on_external_message(client, userdata, message):
    process_message(message, external_client)



def AutopilotService (connection_mode, operation_mode, external_broker, username, password, droneId, simulationPort,log_Flight_Plan_Flag):

    global op_mode
    global external_client
    global internal_client
    global state
    global current_in_sight_traffic
    global current_not_sight_traffic
    global current_network_status
    global current_subnetwork   
    global current_requested_subnet
    global priority 
    global waypoints

    current_in_sight_traffic = []; current_not_sight_traffic=[]; current_network_status = 'free'; current_subnetwork = 'none'; current_requested_subnet =[]
    waypoints = ''
    

    state = 'disconnected'
    print("************************ " + droneId.upper() + " ************************")
    print ('Connection mode: ', connection_mode)
    print ('Operation mode: ', operation_mode)
    op_mode = operation_mode

    #Open internal client
    internal_client_name = "Autopilot_internal_" + droneId
    internal_client = mqtt.Client(internal_client_name)
    internal_client.on_message = on_internal_message
    internal_client.on_connect = on_connect

    #Open external client
    external_client_name = "Autopilot_external_" + droneId
    external_client = mqtt.Client(external_client_name, transport="websockets")
    external_client.on_message = on_external_message
    external_client.on_connect = on_connect

    if connection_mode== "global":
        if external_broker == "hivemq":
            external_client.connect("broker.hivemq.com", 8000)
            print('Connected to broker.hivemq.com:8000')

        elif external_broker == "hivemq_cert":
            external_client.tls_set(ca_certs=None, certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED,
                           tls_version=ssl.PROTOCOL_TLS, ciphers=None)
            external_client.connect("broker.hivemq.com", 8884)
            print('Connected to broker.hivemq.com:8884')

        elif external_broker == "classpip_cred":
            external_client.username_pw_set(
                username, password
            )
            external_client.connect("classpip.upc.edu", 8000)
            print('Connected to classpip.upc.edu:8000')

        elif external_broker == "classpip_cert":
            external_client.username_pw_set(
                username, password
            )
            external_client.tls_set(ca_certs=None, certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED,
                           tls_version=ssl.PROTOCOL_TLS, ciphers=None)
            external_client.connect("classpip.upc.edu", 8883)
            print('Connected to classpip.upc.edu:8883')
        elif external_broker == "localhost":
            external_client.connect("localhost", 8000)
            print('Connected to localhost:8000')
        elif external_broker == "localhost_cert":
            print('Not implemented yet')

    elif connection_mode == "local":
        if operation_mode == "simulation":
            #external_client.connect("localhost", 1884)
            internal_client.connect("localhost", 1884)
            print('Connected to localhost:1884')
        else:
            external_client.connect("10.10.10.1", 8000)
            print('Connected to 10.10.10.1:8000')
    if connection_mode== "global":

        external_client.subscribe("$SYS/broker/clients/connected")
        #external_client.subscribe("traffics/autopilotService_" + droneId + "/#")
        external_client.subscribe(f"autopilotService_{droneId}/{droneId}/#")
        
        #external_client.subscribe("cameraService/+/#", 2)
        external_client.subscribe("+/autopilotService_" + droneId +"/#")
        external_client.subscribe("+/traffic/#")
        external_client.publish(droneId + "/" + "autopilotService_" + droneId + "/connect") #Se envia en cada loop(NO DEBERIA!!!!!)

    elif connection_mode== "local":

        internal_client.subscribe("$SYS/broker/clients/connected")
        #external_client.subscribe("traffics/autopilotService_" + droneId + "/#")
        internal_client.subscribe(f"autopilotService_{droneId}/{droneId}/#")
        
        #external_client.subscribe("cameraService/+/#", 2)
        internal_client.subscribe("+/autopilotService_" + droneId +"/#")
        internal_client.subscribe("+/traffic/#")
        internal_client.publish(droneId + "/" + "autopilotService_" + droneId + "/connect") #Se envia en cada loop(NO DEBERIA!!!!!)
        #internal_client.publish(droneId + "/" + "autopilotService_" + droneId + "/armDrone") #Se envia en cada loop(NO DEBERIA!!!!!)

    time.sleep(2)  

    #------------Initialize connection with simulated drone------------

    if operation_mode == 'simulation':
        print("All connections done, ready to fly")
        if connection_mode== "global":
            external_client.loop_forever()
        if connection_mode== "local":
            internal_client.loop_forever()

    else:
        #external_client.loop_start() #when executed on board use loop_start instead of loop_forever
        print("All connections done, ready to fly")
        external_client.loop_forever()


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 8:
        print("Not enough input arguments to initialize drone")
    
    connection_mode = sys.argv[1] # global or local
    operation_mode = sys.argv[2] # simulation or production
    username = None
    password = None
    if connection_mode == 'global':
        external_broker = sys.argv[3]
        if external_broker == 'classpip_cred' or external_broker == 'classpip_cert':
            username = sys.argv[4]
            password = sys.argv[5]
    else:
        external_broker = None

    global droneId, simulationPort, priority, velocity, conflictRadius,link_Distance,safe_factor,delta_t
    droneId = sys.argv[6]
    simulationPort = sys.argv[7]
    priority = sys.argv[8]
    log_Flight_Plan_Flag = sys.argv[9]
    velocity  = sys.argv[10]
    conflictRadius = sys.argv[11]
    link_Distance = sys.argv[12]
    safe_factor = sys.argv[13]
    delta_t = sys.argv[14]
    time_horizon = sys.argv[15]
    waypoints_file_path = sys.argv[16]


    print(f"Simulation_port: {simulationPort}")

    AutopilotService(connection_mode,operation_mode, external_broker, username, password, droneId, simulationPort,log_Flight_Plan_Flag)