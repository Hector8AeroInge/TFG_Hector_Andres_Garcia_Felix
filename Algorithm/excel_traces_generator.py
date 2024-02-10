import openpyxl
from Common_Functions import distance_Between_Drones, sort_traffic_by_name, get_actual_time
import os

def crate_excel_file(type):
    #***********************************************************************************************
    # Función que genera un archivo excel donde posteriormente se guardaran las trazas del algoritmo
    # Util para la fase de depuración
    #***********************************************************************************************
    #Type: algortihm traces that are being saved
    actual_time = get_actual_time()
    file_name = str( type + "_simulation_traces_" + actual_time +".xlsx")
    Sim_traces_excel = openpyxl.Workbook()

    folder_name = "Simulation_traces"
    py_route = os.path.dirname(__file__)
    folder_route = os.path.join(py_route,folder_name)
    
    file_route = os.path.join(folder_route, file_name)
    Sim_traces_excel.save(file_route)
    return Sim_traces_excel, file_route
    

def add_sheet_excel_traces(file, file_route, sheet_name):
    #***************************************************************************************
    # Función que añade un nueva hoja al archivo y en esta añade los titulos de las columnas
    #***************************************************************************************
    try:
        sheet = file.create_sheet(title=sheet_name)
        titles = ["Algorithm","t","ID","status","Position[x,y,z]","Heading[x,y,z]","Velocity(m/s)","Current Waypoint","RHO_applied"]
        sheet.append(titles)
        file.save(file_route)
    except Exception as e:
        print(f"Traces fail {e}")

    try:
        file.remove(file["Sheet"])
    except:
        pass

def add_single_drone_trace(file, file_route, sheet_name, type, traffic, agent, t):
    #*******************************************************
    # Función que añade una linea de datos al excel generado
    #*******************************************************
    try:
        sheet = file[sheet_name]
        line = [type, t, agent.id, agent.status, str(agent.get_position_as_array()).replace(" "," ").replace(".",". "),
                 str(agent.heading).replace(" ",""), agent.velocity, agent.currentWaypoint, agent.RHO_applied]

        sorted_traffics = sort_traffic_by_name(traffic) #Quiza se puede quitar para ahorrar tiempo de computación en caso de dejar trazas en applicación real
        #Podria oredenarse en el mismo lugar donde se llama la función para no ordenar todo el rato
        for drone in sorted_traffics:
            if drone != agent:
                line.append(distance_Between_Drones(agent,drone))
            else: line.append("-")

        sheet.append(line)
        file.save(file_route)
        
    except Exception as e:
        raise
        print(f"Error al agregar la línea: {str(e)}")
