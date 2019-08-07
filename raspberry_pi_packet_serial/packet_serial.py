from roboclaw import Roboclaw
from time import sleep

if __name__ == "__main__":
    
    address = 0x80
    roboclaw = Roboclaw("/dev/ttyS0", 38400)
    roboclaw.Open()
    
    roboclaw.SetM1VelocityPID(0x81,21.9,15.53,0,375)
    roboclaw.SetM2VelocityPID(0x81,21.9,15.53,0,375)
    while True:
        roboclaw.BackwardM1(0x81,70)
        roboclaw.BackwardM2(0x81,70)
        sleep(3)
        roboclaw.ForwardM1(0x81,55)
        roboclaw.BackwardM2(0x81,55)
        sleep(2)
        roboclaw.BackwardM1(0x81,70)
        roboclaw.BackwardM2(0x81,70)
        sleep(3)
        roboclaw.ForwardM1(0x81,0)
        roboclaw.ForwardM2(0x81,0)
        sleep(10)
        


    

