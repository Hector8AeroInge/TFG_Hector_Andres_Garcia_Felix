# .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
#| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
#| |     ______   | || |   _____      | || |      __      | || |    _______   | || |    _______   | || |  _________   | || |    _______   | |
#| |   .' ___  |  | || |  |_   _|     | || |     /  \     | || |   /  ___  |  | || |   /  ___  |  | || | |_   ___  |  | || |   /  ___  |  | |
#| |  / .'   \_|  | || |    | |       | || |    / /\ \    | || |  |  (__ \_|  | || |  |  (__ \_|  | || |   | |_  \_|  | || |  |  (__ \_|  | |
#| |  | |         | || |    | |   _   | || |   / ____ \   | || |   '.___`-.   | || |   '.___`-.   | || |   |  _|  _   | || |   '.___`-.   | |
#| |  \ `.___.'\  | || |   _| |__/ |  | || | _/ /    \ \_ | || |  |`\____) |  | || |  |`\____) |  | || |  _| |___/ |  | || |  |`\____) |  | |
#| |   `._____.'  | || |  |________|  | || ||____|  |____|| || |  |_______.'  | || |  |_______.'  | || | |_________|  | || |  |_______.'  | |
#| |              | || |              | || |              | || |              | || |              | || |              | || |              | |
#| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
# '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 

import numpy as np
import math
import Common_Functions

class Point:
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Clase para definir los puntos en el espacio con sus atributos
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def __init__(self, x, y, z):
        #*************************
        # Atributos x,y,z tipo float para dar 3 dimensiones a los puntos
        #*************************
        self.x = round(x,10)
        self.y = round(y,10)
        self.z = round(z,10)
            
    def __add__(self, other):
        # Sobrecarga del operador de suma para puntos
        return Point(self.x + other[0], self.y + other[1], self.z + other[2])
    def __mul__(self, scalar):
        # Multiplicación de un punto por un escalar
        return Point(self.x * scalar, self.y * scalar, self.z * scalar)

    def distance_To_Closest_Segment(self, route):
        #***********************************************************************************
        # Función propia que calcula la distancia minima a una lista de segmentos tipo Line.
        #***********************************************************************************
        minDistance = float('inf')

        for line in route:

            distance = line.distance_To_Point(self)
            minDistance = min(minDistance,distance)
        
        return minDistance

    def distance_between_Points(self, point0):
        #******************************************************
        # Función propia que calcula la distancia a otro punto.
        #******************************************************
        dx = self.x - point0.x
        dy = self.y - point0.y
        dz = self.z - point0.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance

    def calculate_unitary_Vector(self,point1):
        #***************************************************************************************
        # Función propia que dado otro punto, calcula el vector unitario con origen en si mismo.
        #***************************************************************************************
        point0_coords = np.array([self.x, self.y, self.z])
        point1_coords = np.array([point1.x, point1.y, point1.z])

        # Calcula el vector entre los puntos
        vector = point1_coords - point0_coords

        # Calcula la magnitud del vector
        magnitude = np.linalg.norm(vector)

        if vector.all() == magnitude.all() == 0:

            return vector

        if vector.all() == np.nan:
            return np.array([0,0,0])
        else:
            # Calcula el vector unitario dividiendo cada componente por la magnitud
            unit_vector = vector / magnitude
            return unit_vector
        
    def pretty_print_point(self):
        #*********************************
        # Función propia para dar un formato de impresión a las componentes del propio punto
        #*********************************
        return str(self.x) + "," + str(self.y) + "," + str(self.z)

class Line:
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #Clase para definir segmentos en el espacio y con sus atributos
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def __init__(self, startPoint, endPoint):
        #****************************************************************************************
        # Inicializador con "startPoint" y "endPoint" con los cuales se puede definir un segmento
        # en el espacio tridimensional
        #****************************************************************************************

        self.startPoint = startPoint
        self.endPoint = endPoint
        self.length = np.linalg.norm(np.array([endPoint.x - startPoint.x, endPoint.y - startPoint.y, endPoint.z - startPoint.z])) #Longitud del segmento
        self.direction = np.array([endPoint.x - startPoint.x, endPoint.y - startPoint.y, endPoint.z - startPoint.z])              #Dirección del segmento
        self.unit_direction = self.direction / self.length if self.length != 0 else np.zeros_like(self.direction)                 #Vector unitario del segmento

    def is_point_on_line(self, waypoint):
        #********************************************************************
        # Función propia que chequea si un punto está contenido en un segmento
        #********************************************************************

        # Calcular el vector desde el punto de inicio de la línea al waypoint
        vector_to_waypoint = np.array([waypoint.x - self.startPoint.x, waypoint.y - self.startPoint.y, waypoint.z - self.startPoint.z])

        # Verificar si el vector es paralelo a la dirección de la línea
        dot_product = np.dot(vector_to_waypoint, self.unit_direction)

        # Verificar si el waypoint está en la misma dirección que la línea
        if 0 <= dot_product <= self.length:

            # El waypoint está en la dirección de la línea, ahora verifica la distancia
            distance_to_waypoint = np.linalg.norm(vector_to_waypoint)
            return distance_to_waypoint <= self.length #Devuelve True si el punto está contenido en el segmento 
        
        else:
            return False #Devuelve False si el punto está contenido en el segmento
        
    def min_Distance_To_Other_Line(self, otherLine):
        #***********************************************************************************
        # Función propia que calcula la minima distancia entre el propio segmento y otro dado
        #***********************************************************************************

        # Calcular el vector entre los puntos de inicio de ambas líneas
        P1_to_P2 = np.array([otherLine.startPoint.x - self.startPoint.x,
                             otherLine.startPoint.y - self.startPoint.y,
                             otherLine.startPoint.z - self.startPoint.z])

        # Calcular la distancia utilizando la fórmula
        numerator = np.linalg.norm(np.cross(P1_to_P2, self.direction))
        denominator = np.linalg.norm(np.cross(self.direction, otherLine.direction))

        # Si las líneas son paralelas, calcular la distancia utilizando un punto de la línea actual
        if denominator == 0:

            w0 = np.array([self.startPoint.x - otherLine.startPoint.x,
                           self.startPoint.y - otherLine.startPoint.y,
                           self.startPoint.z - otherLine.startPoint.z])
            distance = np.linalg.norm(w0 - np.dot(w0, self.direction) * self.direction / np.dot(self.direction, self.direction))

        else:
            distance = numerator / denominator 

        return distance
    
    def distance_To_Point(self, point):
        #**********************************************************************************************
        # Función propia la cual dada un punto calcula la minima distancia desde este al propio segmento
        #**********************************************************************************************

        # Vector from the start point of the line to the given point
        start_to_point = np.array([point.x - self.startPoint.x, 
                                   point.y - self.startPoint.y, 
                                   point.z - self.startPoint.z])

        # Project the start_to_point vector onto the line
        t = np.dot(start_to_point, self.unit_direction)

        # Clamp t to the range [0, length] to handle points outside the line segment
        t = np.clip(t, 0, self.length)

        # Calculate the closest point on the line to the given point
        closest_point = self.startPoint.x + self.unit_direction[0] * t, \
                        self.startPoint.y + self.unit_direction[1] * t, \
                        self.startPoint.z + self.unit_direction[2] * t

        # Calculate the distance between the closest point and the given point
        distance = np.linalg.norm(np.array([point.x - closest_point[0], 
                                            point.y - closest_point[1], 
                                            point.z - closest_point[2]]))

        return distance

class Drone:
    #+++++++++++++++++++++++++++++++++++++++++
    # Clase para definir drones y sus atributos
    #+++++++++++++++++++++++++++++++++++++++++
    def __init__(self, id, position, waypoints, velocity, conflictRadius, priority, currentWaypoint):
        #*********************************************************************************************************************
        # Inicializador con el identificador del dorn, la posición, la lista de waypoints a recorrer, la velocidad de crucero, 
        # el radio de conflicto, el número de prioridad y el numero de waypoint hacia el que se dirige
        #*********************************************************************************************************************

        self.id = id                      #Identificador del dron
        self.position = position          #Posición actual del dron (Class Point)
        self.waypoints = waypoints        #Vector de puntos (Class Point)
        self.path = []                    #Vector de puntos (Class Point)
        self.route = self.generate_route()#Vector de lineas (Class Line)
        self.velocity = float(velocity)   #Velocidad de crucero
        self.startPoint = position        #Posición de inicio de la simulación (Class Point)
        self.endPoint = waypoints[-1]     #Ultimo waypoint a alcanzar (Class Point)
        self.conflictRadius = float(conflictRadius) #Radio minimo de separación
        self.priority = int(priority)     #Prioridad entre 0-5
        self.status = 'on route'          #Estado actual del dron
        self.currentSegment = None        #Actual segmento de la ruta
        self.currentWaypoint = int(currentWaypoint)  #Actual nº de waypoint de la ruta hacia el que se dirige
        self.heading = position.calculate_unitary_Vector(waypoints[int(currentWaypoint)]) #Dirección de velocidad
        self.RHO_applied = False          #Bandera para determinar si ha sufrido regulaciones por RHO
        self.previous_heading = position.calculate_unitary_Vector(waypoints[int(currentWaypoint)])# Dirección de velocidad anterior
        self.solution_waypoints = []      #Vector de puntos que conforman la solución (Class POint) 
        self.restriction_solution = []    #Vector de booleanos que conforman la solución

    def get_position_as_array(self):
        #***************************************************************************************************
        # Función propia la cual devuelve la posición tipo Point de un dron, en un array de la libreria numpy
        #***************************************************************************************************
        array = np.array([self.position.x,self.position.y,self.position.z])
        return array
    
    def generate_route(self):
        #***************************************************************************************************
        # Función propia la cual genera una lista de segmentos en base a los puntos de la lista "waypoints"
        #***************************************************************************************************
        route = []
        for i in range(len(self.waypoints) - 1):

            line = Line(self.waypoints[i], self.waypoints[i + 1])
            route.append(line)

        return route
    
    def update_Position(self,point):
        #*************************************************************
        # Función propia la cual actualiza la posición del objeto Drone
        #*************************************************************
        self.position = point

    def set_Route(self, waypoints):
        #*********************************************************
        # Función propia la cual actualiza la ruta del objeto Drone
        #*********************************************************
        self.route = waypoints

    def update_Path(self, point):
        #*******************************************************************
        # Función propia la cual añade un punto al recorrido del objeto Drone
        #*******************************************************************
        self.path.append(point)
    
    def set_Velocity(self, v):
        #**************************************************************
        # Función propia la cual actualiza la velocidad del objeto Drone
        #**************************************************************
        self.velocity = v

    def set_Path_As_Route(self):
        #*****************************************************************************
        # Función propia la cual actualiza la ruta de un dron por su anterior recorrido
        #*****************************************************************************
        self.route =[]
        lines = []
        for i in range(len(self.path)-1):

            line = Line(self.path[i], self.path[i+1])

            if line.length > 0:

                lines.append(line)

        self.route  = lines

    def set_Path_As_Way_Points(self):
        #******************************************************************************************************
        # Función propia la cual actualiza la lista waypoints de un dron por los puntos de su anterior recorrido
        #******************************************************************************************************
        self.waypoints = []

        for i in range(len(self.path)-1):

            self.waypoints.append(self.path[i])

    def reset(self):
        #***************************************************************************
        # Función propia la cual devuelve a las condiciones iniciales al objeto Drone
        #***************************************************************************
        self.position = self.startPoint
        self.status = 'on route'
        self.currentWaypoint = 0
        self.heading = self.position.calculate_unitary_Vector(self.waypoints[int(self.currentWaypoint)])

    def relative_Position_Between_Drone_And_Point(self, point):
        #*************************************************************************************
        # Función propia la cual devuelve la posición relativa entre un punto y el objeto Drone
        #*************************************************************************************
        relativePosition = np.array([self.position.x - point.x,
                                    self.position.y - point.y,
                                    self.position.z - point.z])
        return relativePosition

    def move(self, delta_t):

        #*************************************************************************************************
        # Función propia la cual actualiza la posición del dron en función de su velocidad y heading actual
        #*************************************************************************************************

        if self.position == self.endPoint:
            
            self.status == 'ended'

        moved = False
        if self.status == 'on route':

            newPosition = Common_Functions.calculate_new_position(self.position,self.heading,self.velocity,delta_t)
            self.previous_heading = self.heading

            if Line(self.position,newPosition).is_point_on_line(self.waypoints[self.currentWaypoint]) == True: #El dron se ha pasado el waypoint

                movedDistance = self.position.distance_between_Points(newPosition)
                remainingDistance = movedDistance - self.position.distance_between_Points(self.waypoints[self.currentWaypoint])

                if remainingDistance == 0:

                    self.position = newPosition
                    moved = True
                    self.currentWaypoint += 1

                    try:

                        if self.currentWaypoint <= len(self.waypoints): 

                            self.heading = self.position.calculate_unitary_Vector(self.waypoints[self.currentWaypoint])

                            if self.heading[0] != self.previous_heading[0] or self.heading[1] != self.previous_heading[1] or self.heading[2] != self.previous_heading[2]:
                                
                                self.solution_waypoints.append(self.position)
                                self.restriction_solution.append(False)

                    except IndexError: 

                        self.position = self.endPoint  
                        moved = True
                        self.status = 'ended' 
                        self.solution_waypoints.append(self.position)
                        self.restriction_solution.append(False)

                    if newPosition.x == self.endPoint.x and newPosition.y == self.endPoint.y and newPosition.z == self.endPoint.z:
                        self.status = 'ended'
                        self.solution_waypoints.append(self.position)
                        self.restriction_solution.append(False)


                elif remainingDistance > 0:

                    while remainingDistance >= 0:

                        self.currentWaypoint += 1
                        preRemainingDistance = remainingDistance
                        

                        try: 
                            remainingDistance = remainingDistance - self.position.distance_between_Points(self.waypoints[self.currentWaypoint])
                        except IndexError:    
                            #Si IndexError, dron ha llegado al final                      
                            self.position = self.endPoint  
                            moved = True
                            self.status = 'ended'
                            self.solution_waypoints.append(self.position)
                            self.restriction_solution.append(False)


                            break       
                    if moved == False:

                        lastWaypoint = self.waypoints[self.currentWaypoint-1]
                        nextWaypoint = self.waypoints[self.currentWaypoint]
                        self.position = lastWaypoint + lastWaypoint.calculate_unitary_Vector(nextWaypoint) * preRemainingDistance
                        self.heading = self.position.calculate_unitary_Vector(self.waypoints[self.currentWaypoint])

                        if self.heading[0] != self.previous_heading[0] or self.heading[1] != self.previous_heading[1] or self.heading[2] != self.previous_heading[2]:
                            
                            self.solution_waypoints.append(self.position)
                            self.restriction_solution.append(False)

                        if self.position == self.endPoint:

                            self.status == 'ended'
                            self.solution_waypoints.append(self.position)
                            self.restriction_solution.append(False)

            else:

                self.position = newPosition
                moved = True
        
        elif self.status == 'avoiding':

            self.position = Common_Functions.calculate_new_position(self.position,self.heading,self.velocity,delta_t)
            self.solution_waypoints.append(self.position)
            self.restriction_solution.append(False)

            moved = True

            if self.position == self.endPoint:

                self.status = 'ended'
    
    
class Conflict:
    #+++++++++++++++++++++++++++++++++++++++++++++
    # Clase para definir conflictos y sus atributos
    #+++++++++++++++++++++++++++++++++++++++++++++
    def __init__(self,conflictedTraffics,t,positions):
        #*******************************************************************************************************************
        # Inicializador formado por el vector que contiene la pareja de traficos en conflicto, el tiempo en el que entran en
        # conflicto y las posiciones en las que lo hacen
        #*******************************************************************************************************************
        self.conflictedTraffics = conflictedTraffics
        self.t = t
        self.positions = positions

class Fence:
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Clase para definir planos/vallas en el espacio que representaran obstaculos o limites del espacio aéreo
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def __init__(self, A, B, C, D):
        #*********************************************************
        # Inicializador formado por cuatro puntos tridimensionales
        #*********************************************************

        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.D = np.array(D)
        self.normal = self.calculate_normal()

    def calculate_normal(self):
        #******************************************************
        # Función propia que devuelve el vector normal al plano
        #******************************************************

        AB = self.B - self.A
        AC = self.C - self.A
        normal = np.cross(AB, AC)
        normalized_normal = normal / np.linalg.norm(normal)
        return normalized_normal

    def distance_to_point(self, point):
        #**************************************************************************
        # Función propia que calcula la distancia mas corta desde un punto al plano
        #**************************************************************************
        return np.abs(np.dot(self.normal, (point - self.A))) / np.linalg.norm(self.normal)
    
    def does_drone_intersect(self, vector):
        #**************************************************************************************
        # Función propia que calcula si una linea de direccion "vector" intersecta con el plano
        #**************************************************************************************

        # Calcula el punto de intersección potencial con el plano
        intersection_point = self.A

        # Calcula el valor de t para encontrar el punto de intersección
        denominator = np.dot(self.normal, vector)
        
        if denominator != 0:
            t = np.dot(self.normal, self.A - vector) / denominator
            intersection_point = vector + t * vector

            # Comprueba si el punto de intersección está dentro de los límites del plano
            min_x = min(self.A[0], self.B[0], self.C[0], self.D[0])
            max_x = max(self.A[0], self.B[0], self.C[0], self.D[0])
            min_y = min(self.A[1], self.B[1], self.C[1], self.D[1])
            max_y = max(self.A[1], self.B[1], self.C[1], self.D[1])
            min_z = min(self.A[2], self.B[2], self.C[2], self.D[2])
            max_z = max(self.A[2], self.B[2], self.C[2], self.D[2])

            if (
                min_x <= intersection_point[0] <= max_x and
                min_y <= intersection_point[1] <= max_y and
                min_z <= intersection_point[2] <= max_z
            ):
                return intersection_point  # Devuelve el punto de intersección si está dentro de los límites del plano
            else:
                return False  # No hay intersección
        else:
            return False
        
    def project_vector_onto_plane(self,vector):
        #*****************************************************
        # Función propia que proyecta un vector sobre el plano
        #*****************************************************

        # Calcula la componente perpendicular al plano
        perpendicular_component = np.dot(vector, self.normal) * self.normal

        # Calcula la proyección del vector sobre el plano
        projected_vector = vector - perpendicular_component

        # Normaliza el vector proyectado para que sea unitario
        normalized_projected_vector = projected_vector / np.linalg.norm(projected_vector)

        return normalized_projected_vector

class Heading:
    #++++++++++++++++++++++++
    # Clase para definir rubos 
    #++++++++++++++++++++++++
    def __init__(self,direction,prio_factor):
        #***************************************************************************************
        # Inicializador formado por por el vector unitario de dirección y un factor de prioridad
        #***************************************************************************************
        self.direction = direction
        self.prio_factor = prio_factor