from roboclaw import Roboclaw
from time import sleep
from threading import Thread
import paho.mqtt.client as mqtt
import time 
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

MQTT_SERVER = "localhost"
MQTT_PATH = "test_channel"

data = "no data"
xCord = [0,0,0]
avgX = 0.0

avgY = 0.0
yCord = [1,2,3]

face_array = [0,0,0]
avgFace =0

face_growth = [0,0,0]
face_move = ""

tf_name = ""
tf_xPos = 0
tf_yPos = 0
tf_face_size = 0;
avgMovementX = 400
movementX = [1,2,3]
motor_direction = "unsure"
action = False

start_face_distance = 0
start_face_needed = False

currentDetection = "face"

startTime = time.time()
startTimeCup = time.time()
distanceString = ''
distance = 0.0
needToSleep = False
firstRun = True


servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization
p.ChangeDutyCycle(6)
time.sleep(0.1)
p.ChangeDutyCycle(0)
hotwordTime = 0

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
    client.subscribe("voice")
 
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global data
    global xCord
    global avgX
    global yCord
    global avgY
    global tf_name
    global tf_xPos
    global tf_yPos
    global action
    global tf_face_size
    global avgFace
    global face_array
    global avgMovementX
    global face_growth
    global movementX
    global motor_direction
    global start_face_distance
    global face_move
    global currentDetection
    global start_face_needed
    global startTime
    global startTimeCup
    global distance
    global distanceString
    global hotwordTime
    global firstRun
    global needToSleep
    tf_in = (str(msg.payload))
    if (tf_in.find('robot') != -1):
        hotwordTime = time.time() +10
        GPIO.output(21, GPIO.HIGH)
        GPIO.output(20, GPIO.LOW)
        print "voice start"

    if (tf_in.find('coco') != -1):
        #print ("coco")
        if (tf_in.find("distance") != -1):
            length = len(tf_in)
            pos1 = tf_in.find(':')  # split up the input string
            distanceString = tf_in[(pos1+1):(length)]  # this will give you the width of the person
            distance = int(distanceString)
            #print (distance)
        if (tf_in.find("cup") != -1) and (time.time()<hotwordTime):
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(21, GPIO.LOW)
            roboclaw.ForwardM2(address,0)
            roboclaw.BackwardM1(address,0)
            currentDetection = "cup"
            startTimeCup = time.time()
            p.ChangeDutyCycle(8)
            time.sleep(0.5)
            p.ChangeDutyCycle(0)
            print("cup")
            # need to reset face_array[], avg face, yCord[], avgY, Xcord[], avgX
            face_array[0] = 0
            face_array[1] = 0
            face_array[2] = 0
            avgFace = 0
            yCord[0] = 0
            yCord[1] = 0
            yCord[2] = 0
            avgY = 0
            xCord[0] = 300
            xCord[1] = 300
            xCord[2] = 300
            avgX = 300
            motor_direction = "unsure"
            firstRun = False #prevents the robot from randomly spinning in circles when first booted up
            needToSleep = True
        elif(tf_in.find("banana") != -1) and (time.time()<hotwordTime):
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(21, GPIO.LOW)
            roboclaw.ForwardM2(address,0)
            roboclaw.BackwardM1(address,0)
            currentDetection = "banana"
            startTimeCup = time.time()
            p.ChangeDutyCycle(8)
            time.sleep(0.5)
            p.ChangeDutyCycle(0)
            print("cup")
            # need to reset face_array[], avg face, yCord[], avgY, Xcord[], avgX
            face_array[0] = 0
            face_array[1] = 0
            face_array[2] = 0
            avgFace = 0
            yCord[0] = 0
            yCord[1] = 0
            yCord[2] = 0
            avgY = 0
            xCord[0] = 300
            xCord[1] = 300
            xCord[2] = 300
            avgX = 300
            motor_direction = "unsure"
            firstRun = False #prevents the robot from randomly spinning in circles when first booted up
            needToSleep = True
#        Replace the COCO object you want to detect by replacing this keyword **INSERTDETECTION** with your COCO keyword such as 'banana'
#        elif(tf_in.find("**INSERTDETECTION**") != -1) and (time.time()<hotwordTime):
#            GPIO.output(20, GPIO.HIGH)
#            GPIO.output(21, GPIO.LOW)
#            roboclaw.ForwardM2(address,0)
#            roboclaw.BackwardM1(address,0)
#            currentDetection = "**INSERTDETECTION**"
#            #startTimeCup = time.time()
#            p.ChangeDutyCycle(8)
#            time.sleep(0.5)
#            p.ChangeDutyCycle(0)
#            print("cup")
#            # need to reset face_array[], avg face, yCord[], avgY, Xcord[], avgX
#            face_array[0] = 0
#            face_array[1] = 0
#            face_array[2] = 0
#            avgFace = 0
#            yCord[0] = 0
#            yCord[1] = 0
#            yCord[2] = 0
#            avgY = 0
#            xCord[0] = 300
#            xCord[1] = 300
#            xCord[2] = 300
#            avgX = 300
#            motor_direction = "unsure"
#            firstRun = False #prevents the robot from randomly spinning in circles when first booted up
#            needToSleep = True   
    if (tf_in.find('face') != -1) and (time.time()<hotwordTime):
#        startTime = time.time()
        GPIO.output(20, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW)
        p.ChangeDutyCycle(6)
        time.sleep(0.5)
        p.ChangeDutyCycle(0)
        currentDetection = "face"
        action = not action
        print("follow")
        if (avgFace > 100):
            start_face_distance = avgFace
        else:
            start_face_needed = True
        # need to reset face_array[], avg face, yCord[], avgY, Xcord[], avgX
        face_array[0] = 0
        face_array[1] = 0
        face_array[2] = 0
        avgFace = 0
        yCord[0] = 0
        yCord[1] = 0
        yCord[2] = 0
        avgY = 0
        xCord[0] = 0
        xCord[1] = 0
        xCord[2] = 0
        avgX = 0
        motor_direction = "unsure"
        firstRun = False #prevents the robot from randomly spinning in circles when first booted up
        needToSleep = True#wait for image data to roll in
    if((tf_in.find('person') !=-1) and (currentDetection == "face")):
        startTime = time.time()
        length = len(tf_in)
        pos1 = tf_in.find(':')  # split up the input string
        pos2 = tf_in.find(';')
        pos3 = tf_in.find('@')
        pos4 = tf_in.find('#')
        width = tf_in[(pos1+1):pos2]  # this will give you the width of the person
        width = int(width)
        height = tf_in[(pos2 + 1):pos3]  # this will give you the height of the person
        height = int(height)
        tf_face_size = width * height
        face_array[2] = face_array[1]
        face_array[1] = face_array[0]
        face_array[0] = tf_face_size
        avgFace = face_array[2] + face_array[1] + face_array[0]
        avgFace = avgFace / 3.0
        #print("hi3")
        


        tf_xPos = float(tf_in[(pos3 + 1):pos4])
        tf_yPos = float(tf_in[(pos4+1):length])
        yCord[2] = yCord[1]
        yCord[1] = yCord[0]
        yCord[0] = tf_yPos
        avgY = yCord[2] + yCord[1] + yCord[0]
        avgY = avgY / 3.0
        xCord[2] = xCord[1]
        xCord[1] = xCord[0]
        xCord[0] = tf_xPos
        avgX = xCord[2] + xCord[1] + xCord[0]
        avgX = avgX / 3.0
        #print("avg size ", avgFace, "xpos ", tf_xPos, "ypos ", tf_yPos,)
        
        if start_face_needed == True:
            start_face_distance = tf_face_size
            start_face_needed = False
        
    if((tf_in.find('cup') !=-1) and (currentDetection == "cup")):
        startTimeCup = time.time()
        length = len(tf_in)
        pos1 = tf_in.find(':')  # split up the input string
        pos2 = tf_in.find(';')
        pos3 = tf_in.find('@')
        pos4 = tf_in.find('#')
        width = tf_in[(pos1+1):pos2]  # this will give you the width of the person
        width = int(width)
        height = tf_in[(pos2 + 1):pos3]  # this will give you the height of the person
        height = int(height)
        tf_face_size = width * height
        face_array[2] = face_array[1]
        face_array[1] = face_array[0]
        face_array[0] = tf_face_size
        avgFace = face_array[2] + face_array[1] + face_array[0]
        avgFace = avgFace / 3.0
        
        tf_xPos = float(tf_in[(pos3 + 1):pos4])
        tf_yPos = float(tf_in[(pos4+1):length])
        yCord[2] = yCord[1]
        yCord[1] = yCord[0]
        yCord[0] = tf_yPos
        avgY = yCord[2] + yCord[1] + yCord[0]
        avgY = avgY / 3.0
        xCord[2] = xCord[1]
        xCord[1] = xCord[0]
        xCord[0] = tf_xPos
        avgX = xCord[2] + xCord[1] + xCord[0]
        avgX = avgX / 3.0
    if ((tf_in.find('banana') !=-1) and (currentDetection == "banana")):
        startTimeCup = time.time()
        length = len(tf_in)
        pos1 = tf_in.find(':')  # split up the input string
        pos2 = tf_in.find(';')
        pos3 = tf_in.find('@')
        pos4 = tf_in.find('#')
        width = tf_in[(pos1+1):pos2]  # this will give you the width of the person
        width = int(width)
        height = tf_in[(pos2 + 1):pos3]  # this will give you the height of the person
        height = int(height)
        tf_face_size = width * height
        face_array[2] = face_array[1]
        face_array[1] = face_array[0]
        face_array[0] = tf_face_size
        avgFace = face_array[2] + face_array[1] + face_array[0]
        avgFace = avgFace / 3.0
        print("avg size ", avgFace, "xpos ", tf_xPos, "ypos ", tf_yPos,)
        
        tf_xPos = float(tf_in[(pos3 + 1):pos4])
        tf_yPos = float(tf_in[(pos4+1):length])
        yCord[2] = yCord[1]
        yCord[1] = yCord[0]
        yCord[0] = tf_yPos
        avgY = yCord[2] + yCord[1] + yCord[0]
        avgY = avgY / 3.0
        xCord[2] = xCord[1]
        xCord[1] = xCord[0]
        xCord[0] = tf_xPos
        avgX = xCord[2] + xCord[1] + xCord[0]
        avgX = avgX / 3.0
        


    if avgX == 0:
        motor_direction == "unsure"
    elif avgX>470:
        motor_direction = "left"
    elif avgX<170:
        motor_direction = "right"
    elif avgX <470 and avgX >170:
        motor_direction = "neither"
    if (currentDetection == "face"):
        if (avgFace > (start_face_distance + 8000)): #100 is a filter
            face_move = "further"
        elif (avgFace < (start_face_distance -8000)):
            face_move = "closer"
        else:
            face_move = ""
    t = 0


        
client = mqtt.Client()
client.on_connect = on_connect
client.connect(MQTT_SERVER, 1883, 60)

def runB():
    #client = mqtt.Client()
    #client.on_connect = on_connect
    client.on_message = on_message
    #client.connect(MQTT_SERVER, 1883, 60)
    client.loop_forever()




if __name__ == "__main__":
    t2 = Thread(target = runB)
    t2.setDaemon(True)
    t2.start()
    
    old = data
    address = 0x81
    roboclaw = Roboclaw("/dev/ttyS0", 38400)
    roboclaw.Open()
    roboclaw.SetM2VelocityPID(0x80,1.746,0.25225,0,6000)
    roboclaw.SetM1VelocityPID(0x81,21.9,15.53,0,375)
    roboclaw.SetM2VelocityPID(0x81,21.9,15.53,0,375)
    sleep(1)
    roboclaw.SetM2VelocityPID(0x80,1.746,0.25225,0,6000)
    roboclaw.SetM1VelocityPID(0x81,21.9,15.53,0,375)
    roboclaw.SetM2VelocityPID(0x81,21.9,15.53,0,375)
    

    while True:
        if (needToSleep == False):
            if hotwordTime<time.time():
                GPIO.output(20, GPIO.LOW)
                GPIO.output(21, GPIO.LOW)
                print "voice timeout"             
            
            if (currentDetection == "face"):
                if (time.time() > startTime + 10):
                    motor_direction = "unsure"
                if (motor_direction == "left"):
                    print ("R")
                    try:
                        roboclaw.ForwardM1(address,40)
                        roboclaw.BackwardM2(address,40)
                    except:
                        print("")
                elif (motor_direction == "right"):
                    print ("L")
                    try:
                        roboclaw.ForwardM2(address,40)
                        roboclaw.BackwardM1(address,40)
                    except:
                        print("")
                    #sleep(0.3)
                    #motor_direction = ""
                elif (motor_direction == "unsure" and firstRun == False):
                    print("rotating")
                    roboclaw.BackwardM1(address,60)
                    roboclaw.ForwardM2(address,60)
                else:
                    print("N")
                    if (face_move == 'further'):
                        print("MAIN further")
                        try:
                            roboclaw.ForwardM1(address,40)
                            roboclaw.ForwardM2(address,40)
                        except:
                            print("")
                    elif(face_move =='closer'):
                        print("MAIN closer")
                        try:
                            roboclaw.BackwardM1(address,40)
                            roboclaw.BackwardM2(address,40)
                        except:
                            print("")
                    else:
                        print('N')
                        try:
                            roboclaw.ForwardM2(address,0)
                            roboclaw.BackwardM1(address,0)
                        except:
                            print("")

            if (currentDetection == "cup" or currentDetection == "banana"): #you can add your additional detection objects here
                if (time.time() > startTimeCup + 10):
                    motor_direction = "unsure"
                if (motor_direction == "left"):
                    print ("R")
                    roboclaw.ForwardM1(address,35)
                    roboclaw.BackwardM2(address,35)
                    #sleep(0.3)
                    #motor_direction = ""
                elif (motor_direction == "right"):
                    print ("L")
                    roboclaw.ForwardM2(address,35)
                    roboclaw.BackwardM1(address,35)
                    #sleep(0.3)
                    #motor_direction = ""
                elif (motor_direction == "unsure"):
                    print("rotating")
                    roboclaw.BackwardM1(address,40)
                    roboclaw.ForwardM2(address,40)
                elif motor_direction == "neither":
                    print("Neutral")
                    print (distance)
                    roboclaw.SpeedDistanceM1M2(address,0,0,0,0,1)
                    if (distance >35):
                        roboclaw.SpeedDistanceM1M2(0x81,-150,10,-150,10,1)#replace with something more bulletfproof
                    if (distance > 20 and distance <35):
                        print ('pickup object')                    
                        #roboclaw.BackwardM1M2(0x81,64)
                        #distanceMove = float(distance/35) * 150 #integrate a better value for how far
                        roboclaw.SpeedDistanceM2(0x80,-1000,500,1)
                        roboclaw.SpeedDistanceM1M2(0x81,-150,300,-150,300,1)
                        sleep(3)
                        roboclaw.SpeedM1M2(0x80,0,0)
                        roboclaw.SpeedDistanceM2(0x80,1000,500,1)
                        #rest of the code
                        sleep(3)
                        roboclaw.SpeedM1M2(0x80,0,0)
                        roboclaw.SpeedM1M2(0x81,0,0)
                        currentDetection = "face"
                        client.publish("voice", "robot")
                        sleep(1)
                        client.publish("voice", "face")
                        p.ChangeDutyCycle(6)
                        time.sleep(0.5)
                        p.ChangeDutyCycle(0)
                        sleep(8)#wait for image data to roll in
                        #find an easy way to do the whole reset of data points and servo angle done by the on message part.                 
                    else:
                        print('BUG')
        if (needToSleep == True):
            roboclaw.BackwardM1(address,0)
            roboclaw.BackwardM2(address,0)
            sleep(10)
            needToSleep = False
            
