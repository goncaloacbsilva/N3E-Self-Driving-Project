import serial
import string
import json
import random

def randomStringDigits(stringLength=6):
    """Generate a random string of letters and digits """
    lettersAndDigits = string.ascii_letters + string.digits
    return ''.join(random.choice(lettersAndDigits) for i in range(stringLength))

class Car(object):
    def __init__(self):
        print("Initializing Car...")
        #Init car
        #Car config
        self.steering = 45
        self.speed = 0
        self.direction = 0
        print("Opening Serial Port...")
        self.device = serial.Serial('/dev/ttyACM0', 115200)
        print("Serial port opened (BAUD: 115200)")
        self.update()

    def set_speed(self, speed):
        self.speed = speed

    def set_steering(self, steering):
        self.steering = steering

    def set_direction(self, direction):
        self.direction = direction

    def send(self):
        ir_code = randomStringDigits();
        self.device.write(str("GET:"+ir_code+"&"+'\n').encode('UTF-8'))
        while True:
            data = self.device.readline()[:-2]
            if ir_code in str(data):
                print(data)
                break

    def update(self):
        dataPKT = "POST:"+str(self.speed)+"&"+str(self.direction)+"S"+str(self.steering)
        print(dataPKT)
        self.device.write(str(dataPKT+'\n').encode('UTF-8'))
        print("Post Complete")

    def cleanup(self):
        print("Closing Serial Port...")
        self.device.close()
