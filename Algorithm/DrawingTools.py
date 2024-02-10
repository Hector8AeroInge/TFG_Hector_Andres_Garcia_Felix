import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import random

def add_line_to_figure(ax, line, label):
    #*********************************************************************
    # Función que añade lineas a las figuras. Sirve para representar rutas
    #*********************************************************************

    xs = [line.startPoint.x, line.endPoint.x]
    ys = [line.startPoint.y, line.endPoint.y]
    zs = [line.startPoint.z, line.endPoint.z]

    # Agregar la línea a la figura
    ax.plot(xs, ys, zs)

def add_point_to_plot(ax, point, label):
    #**********************************************************************
    # Función que añade puntos a las figuras. Sirve para representar drones
    #**********************************************************************
    ax.scatter(point.x, point.y, point.z, c='r', marker='o')

def add_wall_to_plot(ax,wall,label):
    #*********************************************************************************************
    # Función que añade paredes a las figuras. Sirve para representar limites, vallas u obstáculos
    #*********************************************************************************************
    vertices = [wall.A, wall.B, wall.C, wall.D]
    faces = [[vertices[0], vertices[1], vertices[2], vertices[3]]]
    ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

def plot_Routes(traffics, walls):
    #*****************************************************************************************************
    # Función que crea una figura donde se representan a los drones en su posición y sus respectivas rutas
    #*****************************************************************************************************
     # Crear una gráfica 3D
    fig1 = plt.figure()

    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    plt.ioff()
    plt.show(block=False)

    for drones in traffics:
        add_point_to_plot(ax1,drones.position,f'Drone {drones.id}')
        for lines in drones.route:
            add_line_to_figure(ax1,lines,drones.id)

    for wall in walls:
        add_wall_to_plot(ax1,wall,'wall')


    ax1.legend()

    print("Directorio actual:", os.getcwd())
    plt.savefig('routes1.png')
    
    plt.show()


def get_random_color():
    #**********************************************************************
    # Función que genera un color aleatorio en formato hexadecimal
    #**********************************************************************
    color_hex = "#{:06x}".format(random.randint(0, 0xFFFFFF))
    return color_hex


from matplotlib.colors import hsv_to_rgb

def subnet_color(numero):
    #****************************************************************************
    # Función que genera un color en formato hexadecimal en base a un número dado
    #****************************************************************************
    # Mapear el número a un rango específico de colores usando hsv_to_rgb
    hue = (numero % 360) / 360.0  # Normalizar a valores entre 0 y 1
    saturation = 0.8
    value = 0.8
    rgb_color = hsv_to_rgb((hue, saturation, value))

    # Convertir de [0, 1] a [0, 255] y formatear como color hexadecimal
    hex_color = "#{:02X}{:02X}{:02X}".format(int(rgb_color[0] * 255), int(rgb_color[1] * 255), int(rgb_color[2] * 255))

    return hex_color