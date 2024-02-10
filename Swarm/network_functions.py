#NETWORK FUNCTIONS:
#Checekear si existe algun topic con ese nombre. Primero se generará un topic para cada conjunto de nodos
#antes de crearse ese topprobar si e nombre generado aleatoriamente existe ya o no

import paho.mqtt.client as mqtt
import random
import string

def check_topic_existence(client, topic_to_check):
    #************************************************************
    # Función que comprueba si el tema existe en el servidor MQTT
    #************************************************************
    try:
        # Intentar publicar un mensaje en el tema
        result = client.publish(topic_to_check, "Check existence", qos=1)

        # Manejar el resultado de la publicación
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            return True
        elif result.rc == mqtt.MQTT_ERR_NO_CONN:
            print(f"No hay conexión al servidor MQTT. Intentando reconectar...")
            client.reconnect()
            return False
        elif result.rc == mqtt.MQTT_ERR_INVALID_TOPIC:
            print(f"El tema '{topic_to_check}' es inválido. Intentando crearlo...")
            client.subscribe(topic_to_check)  # Intentar suscribirse al tema (crearlo)
            return False
        else:
            print(f"Error desconocido al intentar verificar el tema: {result.rc}")
            return False
    except Exception as e:
        print(f"Error al intentar verificar el tema: {e}")
        return False


def generate_random_numeric_topic(prefix="subnets", length=8):
    #**********************************************************************************************************
    # Función que genera un número aleatorio de la longitud espceificada, el cual será el nombre de las subNets
    #**********************************************************************************************************
    chars = string.digits
    random_string = ''.join(random.choice(chars) for _ in range(length))
    return f"{random_string}"


def generate_sub_network(client):
    #***************************************************************
    # Función que genera una nueva subNet en el cliente especificado
    #***************************************************************
    subNet_topic_valid = False

    while subNet_topic_valid == False:
        subNet_topic = generate_random_numeric_topic(prefix='subnets',length=16)
        subNet_topic_valid = check_topic_existence(client, subNet_topic)

    return subNet_topic

#def add_to_node_group


def custom_priority(str1, str2):
    #***************************************************************************************
    # Función que determina la prioridad en base a los identificadores(strings) de cada dron
    #***************************************************************************************
    if str1 < str2:
        return -1
    elif str1 > str2:
        return 1
    else:
        print('Same droneID detected')
        return 0
        