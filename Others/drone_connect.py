import paho.mqtt.client as mqtt
import json
import time

droneId = "drone00000"
print("Starting " + droneId)

def on_connect(external_client, userdata, flags, rc):
    if rc==0:
        print("Connection OK")
    else:
        print("Bad connection")

def process_message(message, client):
    global vehicle
    global direction
    global go
    global sending_telemetry_info
    global sending_topic
    global op_mode
    global state

    splited = message.topic.split("/")
    origin = splited[0]
    command = splited[2]
    sending_topic = "autopilotService/" + droneId
    print ('Recibo ', command)

    if command == 'telemetryInfo':
        telemetry_info = json.loads(message.payload)
        print(telemetry_info)

def on_external_message(client, userdata, message):
    global external_client
    print(message)
    process_message(message, external_client)


external_client = mqtt.Client(droneId, transport="websockets")
external_client.on_connect = on_connect
external_client.on_message = on_external_message

external_client.username_pw_set("dronsEETAC", "mimara1456.")
external_client.connect('classpip.upc.edu', 8000)

external_client.subscribe('autopilotService_' + droneId + '/' + droneId + '/#', 0)
external_client.subscribe('autopilotService_ ' + droneId + '/newTraffic/#', 0)

external_client.publish(droneId + '/autopilotService_' + droneId + '/newTraffic', droneId)
external_client.publish(droneId + '/autopilotService_' + droneId + '/connect','',0)
print("Waiting data")

external_client.loop_forever()

#loop_start