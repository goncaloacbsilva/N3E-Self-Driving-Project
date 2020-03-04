import matlab.engine
import rospy
import time
from std_msgs.msg import String


def callback(data):

    msg = data.data


# Initialize matlab
eng = matlab.engine.start_matlab()
# function that initializes the control model
eng.init(nargout=0)
# function that activates deactivate the compilation of the simulink model
eng.fastMode(nargout=0)
# function that executes the simulation of the simulink model with the purpose of initializing the model
eng.simModel(nargout=0)
# get the mpc time step for later use
timeStep = eng.workspace['timeSample']

# receive polynomial function, lateral deviation from center line,
# and velocity from camera node and send to matlab workspace

pub = rospy.Publisher('Control_in', String, queue_size=1)
rospy.init_node('normal_control_node')

while True:

    start = time.time()

    rospy.Subscriber("LineInfo", String, callback)

    a, b, c, latdev = msg.split("/")

    # Send the polynomial fuction to the matlab workspace
    eng.workspace['a'] = float(a)
    eng.workspace['b'] = float(b)
    eng.workspace['c'] = float(c)

    # Run curvature script to calculate curvature and load it on matlab workspace
    eng.curv(nargout=0)

    # Send some parameters to the matlab workspace
    eng.workspace['latDev'] = float(latdev)

    # Run the simulink model and get the ideal steerAng
    eng.simModel(nargout=0)
    steerAng = eng.workspace['steerAng']

    pub.publish('0/' + str(int(steerAng)))

    # Timer
    end = time.time()
    t = end - start
    if t < timeStep:
        time.sleep(timeStep-t)

    if rospy.is_shutdown():
        break

