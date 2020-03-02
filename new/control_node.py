import matlab.engine
import time

"""Initialize matlab"""
eng = matlab.engine.start_matlab()

"""fuction that initializes the control model"""
eng.init(nargout=0)
eng.fastMode(nargout=0)
eng.simModel(nargout=0)

"""Get the mpc time step for later use"""
timeStep = eng.workspace['timeSample']

"INIT NODE"
"""recieve polinomial fuction, lateral deviation from center line, 
and velocity from camera node and send to matlab workspace"""

totalstart = time.time()

for p in range(100):

    start = time.time()

    """TEST PARAMETERS"""
    a=5.0 + p
    b=-10.0
    c=2.0

    """Send the polinomial fuction to the matlab workspace """
    eng.workspace['a'] = a
    eng.workspace['b'] = b
    eng.workspace['c'] = c

    """Run curvature script to calculate curvature and load it on matlab workspace"""
    eng.curv(nargout=0)

    """eng.workspace['latDev'] = latdev
    eng.workspace['curvature'] = curvature
    eng.workspace['yawDev'] = yawdev"""

    """Run the simulink model and get the ideal steerAng"""
    eng.simModel(nargout=0)
    steerAng = eng.workspace['steerAng']

    print(p)
    print("Steer Angle:")
    print(steerAng)
    end = time.time()
    print("time:")
    print(end-start)

totalend = time.time()
print("total time:")
print(totalend-totalstart)

print("acabei")
