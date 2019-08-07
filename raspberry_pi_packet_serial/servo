import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
MQTT_SERVER = "localhost"
MQTT_PATH = "voice"

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization


 
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(MQTT_PATH)
 
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    if str(msg.payload).find("cup") != -1:
        print "cup"
        p.ChangeDutyCycle(4)
        time.sleep(0.5)
        p.ChangeDutyCycle(0)

    if str(msg.payload).find("face") != -1:
        print "face"
        p.ChangeDutyCycle(8)
        time.sleep(0.5)
        p.ChangeDutyCycle(0)

 
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect(MQTT_SERVER, 1883, 60)
 

client.loop_forever()


