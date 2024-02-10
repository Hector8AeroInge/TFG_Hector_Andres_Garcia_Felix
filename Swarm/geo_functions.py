from geopy.distance import geodesic

def geopy_distance(coord1, coord2):
    #***********************************************************
    # Función que calcula la distancia entre dos coordenadas LLA
    #***********************************************************
    """
    Calcula la distancia geodésica tridimensional entre dos puntos dados por sus coordenadas (latitud, longitud, altitud).

    Parámetros:
    - coord1: Tupla con las coordenadas del punto 1 (latitud, longitud, altitud).
    - coord2: Tupla con las coordenadas del punto 2 (latitud, longitud, altitud).

    Retorna:
    - Distancia geodésica tridimensional en metros.
    """
    distancia = geodesic(coord1, coord2).meters
    return distancia

from pyproj import Proj, transform

def lla_to_ecef(latitude, longitude, altitude_asl):
    #*********************************************************
    # Función que convierte coordenadas LLA a coordenadas ECEF
    #*********************************************************
    # Definir el sistema de coordenadas geodésicas (latitud, longitud, altitud)
    lla_proj = Proj(proj='latlong', datum='WGS84')

    # Convertir de coordenadas geodésicas a ECEF
    x, y, z = transform(lla_proj, Proj(proj='geocent', datum='WGS84'), longitude, latitude, altitude_asl, radians=False)

    return x, y, z

def ecef_to_lla(x, y, z):
    #*********************************************************
    # Función que convierte coordenadas ECEF a coordenadas LLA
    #*********************************************************
    # Definir el sistema de coordenadas geodésicas (latitud, longitud)
    lla_proj = Proj(proj='latlong', datum='WGS84')

    # Convertir de coordenadas ECEF a geodésicas
    longitude, latitude, altitude_asl = transform(Proj(proj='geocent', datum='WGS84'), lla_proj, x, y, z, radians=False)

    return latitude, longitude, altitude_asl