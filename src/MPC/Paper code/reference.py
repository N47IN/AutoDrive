"""
this is  working type_check - or communicating controller
emitter is at 8

"""

import math
import numpy as np
from vehicle import Driver


ts = 0.1
time_step = int(1000*ts)

driver = Driver()
acelo = driver.getDevice("accelerometer")
acelo.enable(time_step)
driver.setGear(1)
# receiver = driver.getDevice("receiver")
emitter = driver.getDevice("emitter")
gps = driver.getDevice("gps")

ts = 0.1
time_step = int(1000*ts)

# receiver.enable(time_step)
gps.enable(time_step)  ##  number corresponds to frequency
# emitter.enable(time_step)

previous_message = ''

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
a = 0
v = 8
d = 50
print("reference controller is running") 

while driver.step() != -1:
    track = driver.getTime()
    a += 1
    #driver.setCruisingSpeed(60)
    
    # if a > 300 :
        # driver.setThrottle(0.5)
        # if a > 1000:
            # driver.setCruisingSpeed(0)
        # else:
            # driver.setCruisingSpeed(60)
    k = 0.137
    freq = 0.5
    
   
    Accel = k*math.sin(2*math.pi*freq*track)
    ah = acelo.getValues()
    #print("accel is", ah)
   # print(track)
    if Accel > 0 :
      driver.setBrakeIntensity(0)
      driver.setThrottle(Accel)
      
    if Accel < 0 :
      driver.setBrakeIntensity(-Accel)
    gps_car = gps.getValues()
    xh = gps_car[0]
    vh = gps.getSpeed()
    if math.isnan(vh):
        vh = 0
    if math.isnan(xh):
        xh = 0

    xc = rnd(np.array([xh, vh, 0,ah[0]]) )# referance state
    # print("this is xc" , xc)

    message = xc.tobytes()
    #send reference
    if message != '' : # and message != previous_message:
        previous_message = message
        emitter.send(message)

