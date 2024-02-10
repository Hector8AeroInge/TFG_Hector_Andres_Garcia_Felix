#Secondary functions
from Common_Functions import * 

def calculate_Route_Time(velocity,Route,t_0): #(velocity {m/s})
    t=t_0
    for Line in Route:
        t = (Line.length/velocity) + t
    return t

def calculate_time_for_line(line, velocity, t_0):
    # Calcula el tiempo para cada punto en la línea
    times = [line.length * t / velocity + t_0 for t in np.linspace(0, 1, 100)]
    return times

def calculate_obstacle_velocity(drone, nextDrone, relative_position):
    
    distance = distance_Between_Drones(drone,nextDrone)
    minDistance = min([drone.conflictRadius,nextDrone.conflictRadius])
            
    if distance < minDistance:
        # Los agentes están en curso de colisión, calcula la nueva velocidad
        desired_velocity = relative_position / distance * drone.maxVelocity
        obstacle_velocity = desired_velocity - drone.velocity * drone.heading
        return obstacle_velocity / np.linalg.norm(obstacle_velocity) * drone.velocity

    return np.zeros(3)  # No hay colisión, devuelve una velocidad nula

def calculate_rvo_direction(drone, rvoGroup):
    neighbor_dist = drone.conflictRadius * 2  # Distancia de vecindario
    time_horizon = drone.maxVelocity/drone.conflictRadius  # Horizonte de tiempo
    #max_neighbors = len(rvoGroup)    # Número máximo de vecinos considerados
    
    # Variables para calcular la nueva dirección
    preferred_heading = relative_Position_Between_Drone_And_Point(drone,drone.route[drone.currentSegment].endPoint)
    preferred_heading_Norm = preferred_heading/np.linalg.norm(preferred_heading)
    new_velocity = drone.velocity * preferred_heading_Norm

    for nextDrone in rvoGroup:
        if nextDrone != drone:
            relative_position = relative_Position_Between_Drones(drone,nextDrone)
            distance = distance_Between_Drones(drone,nextDrone)

            if distance < neighbor_dist:
                # Vecino está dentro del vecindario
                new_velocity += calculate_obstacle_velocity(drone, nextDrone, relative_position)

    # Aplica la nueva dirección RVO
    new_direction = new_velocity / np.linalg.norm(new_velocity)
    drone.heading = new_direction
    drone.velocity = np.linalg.norm(new_velocity)


def solve_RVO(rvoGroup,delta_t):
    for drone in rvoGroup[0]: 
        drone.status = 'avoiding'
    t=0
    while all(agents.status == 'ended' for agents in  rvoGroup[0]) == False:

        for drone in rvoGroup[0]:
            calculate_rvo_direction(drone,rvoGroup[0])

        move_All_Drones(rvoGroup[0],delta_t,[],t)
        print('t: ', t, 'Distance: ', distance_Between_Drones(rvoGroup[0][0],rvoGroup[0][1]), ' ',rvoGroup[0][0].id,rvoGroup[0][1].id)
        t += 1

def solve_Parallel_Routes2(conflictedPairTraffics,delta_t):
    trafficsSorted = sorted(conflictedPairTraffics, key=lambda x: x.priority)
    slaveDrone = trafficsSorted[0]; masterDrone = trafficsSorted[1]
    minDistance = min([slaveDrone.conflictRadius,masterDrone.conflictRadius])
    slaveDrone.path = []
    restrictions = []

    t=0
    while all(drone.status == 'ended' for drone in conflictedPairTraffics) == False:
        while True:
            distance = distance_Between_Drones(slaveDrone, masterDrone)
            if distance < minDistance:
                restrictions.append(1)  #1 para marcar que en ese delta_t el dornSlave ha de esquivar
                break
            else:
                restrictions.append(0)
