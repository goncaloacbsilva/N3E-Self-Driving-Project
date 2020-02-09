import time
import CarControl
import random
global car

print("Starting Test...")
car = CarControl.Car()

def speed_test(t):
    print("Starting Speed at 0 PWM")
    car.set_speed(0)
    car.update()
    for speed in range(10, 220, 20):
        time.sleep(random.randint(1, t))
        car.set_speed(speed)
        car.update()
        print("Increasing Speed to " + str(speed) + " PWM")

def direction_test():
    car.set_direction(3)
    car.set_steering(0)
    time.sleep(1)
    for steer in range(0, 90, 10):
        print("Testing " + str(steer) + "/" + str(steer+10))
        time.sleep(1)
        for i in range(4):
            car.set_steering(steer)
            car.update()
            time.sleep(1)
            car.set_steering(steer + 30)
            car.update()

def send_test():
    print("Querying IR System...")
    for i in range(80):
        car.send()
        time.sleep(0.05)

def serial_com_test(interval):
    print("Initializing Serial COM Two Ways...")
    print("Testing response time for: " + str(interval) + "s")
    for i in range(150):
        chs = random.randint(0, 50);
        speeds = random.randint(20, 80);
        if chs > 40:
            #print("Increasing Speed to " + str(speeds) + " PWM")
            car.set_speed(speeds)
            car.set_direction(0)
            car.set_steering(45)
            car.update()
        time.sleep(interval)
        #print("Querying IR System...")
        car.send()
    print("Awaiting for shutdown...")
    time.sleep(2)
    car.set_speed(0)
    car.set_direction(3)
    car.update()



while True:
    cmd = input("> ")
    if cmd == "EXIT":
        break
    elif cmd == "SEND_TEST":
        send_test()
    elif cmd == "SPEED_TEST":
        speed_test(2)
    elif cmd == "SCT":
        serial_com_test(0.05)
    else:
        cmd = cmd.split("/")
        car.set_steering(cmd[2])
        car.set_direction(cmd[1])
        car.set_speed(cmd[0])
        car.update()
car.cleanup()
