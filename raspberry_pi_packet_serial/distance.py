# Import necessary libraries.
from time import sleep
 
from Bluetin_Echo import Echo

import paho.mqtt.publish as publish

MQTT_SERVER = "localhost"
MQTT_PATH = "test_channel"

# Define pin constants
TRIGGER_PIN_1 = 24
ECHO_PIN_1 = 8

 
# Initialise two sensors.
echo = [Echo(TRIGGER_PIN_1, ECHO_PIN_1)]
 
def main():
    sleep(0.1)
    for counter in range(1, 2):
        for counter2 in range(0, len(echo)):
            result = echo[counter2].read('cm', 3)
            print (result)
            result2 = int (result)
            if result2 > 0:
                publish.single(MQTT_PATH, 'coco distance:' + str (result2), hostname=MQTT_SERVER)

 
if __name__ == '__main__':
    while True:
        main()

     

