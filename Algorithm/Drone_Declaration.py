import classes

#****************************************************Definir dron_0****************************************************
id = "D0"; velocity = 2; conflictRadius = 4; 
startPoint = classes.Point(3.,0.,6.); endPoint = classes.Point(16., 30., 6.); route = [classes.Line(startPoint,endPoint)]; 
waypoints = [startPoint,endPoint]; position = startPoint; path=[]; 
priority = 1; status = 'on route'; heading = route[0].unit_direction; 
currentSegment = 0; currentWaypoint = 1; RHO_applied = False

Drone_0 = classes.Drone(id, position, path, route, waypoints,velocity, 
                        startPoint, endPoint, conflictRadius, 
                        priority, status, heading, 
                        currentSegment, currentWaypoint,RHO_applied)


#****************************************************Definir dron_1****************************************************
id = "D1"; velocity = 2; conflictRadius = 4; 
startPoint = classes.Point(9.,30.,8.); endPoint = classes.Point(9., 0., 8.); route = [classes.Line(startPoint,endPoint)]; 
waypoints = [startPoint,endPoint]; position = startPoint; path=[]; 
priority = 4; status = 'on route'; heading = route[0].unit_direction; 
currentSegment = 0; currentWaypoint = 1; RHO_applied = False

Drone_1 = classes.Drone(id, position, path, route, waypoints, velocity, 
                        startPoint, endPoint, conflictRadius, 
                        priority, status, heading, 
                        currentSegment, currentWaypoint, RHO_applied)

#****************************************************Definir dron_2****************************************************
id = "D2"; velocity = 2; conflictRadius = 4; 
startPoint = classes.Point(11.,0.,6.); endPoint = classes.Point(11., 30., 6.); route = [classes.Line(startPoint,endPoint)]; 
waypoints = [startPoint,endPoint]; position = startPoint; path=[]; 
priority = 2; status = 'on route'; heading = route[0].unit_direction; 
currentSegment = 0; currentWaypoint = 1; RHO_applied = False

Drone_2 = classes.Drone(id, position, path, route, waypoints, velocity, 
                        startPoint, endPoint, conflictRadius, 
                        priority, status, heading, 
                        currentSegment, currentWaypoint, RHO_applied)


#****************************************************Definir dron_3****************************************************
id = "D3"; velocity = 2; conflictRadius = 4; 
startPoint = classes.Point(10.,30.,15.); endPoint = classes.Point(0., 0., 0.); route = [classes.Line(startPoint,endPoint)]; 
waypoints = [startPoint,endPoint]; position = startPoint; path=[]; 
priority = 1; status = 'on route'; heading = route[0].unit_direction; 
currentSegment = 0; currentWaypoint = 1; RHO_applied = False

Drone_3 = classes.Drone(id, position, path, route, waypoints, velocity, 
                        startPoint, endPoint, conflictRadius, 
                        priority, status, heading, 
                        currentSegment, currentWaypoint, RHO_applied)



import numpy as np
A = np.array([17,-10, 0])
B = np.array([17,-10,10])
C = np.array([17, 40,10])
D = np.array([17, 40, 0])

wall_1 = classes.Fence(A,B,C,D)