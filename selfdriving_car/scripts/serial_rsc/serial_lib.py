#Python library for conversions

def convert_speed(speed):
    y = 37 * speed + 15
    if speed > 2:
        print("SERIAL # WARNING: Speed is now " + str(speed) + "m/s. Be careful")
    return y;

def convert_slope(angle):
    angle = 45 - angle
    if angle < 0:
        angle = 0
    elif angle > 90:
        angle = 90
    else:
        pass
    return angle;
