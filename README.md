Use latest version of controllers!
# AutoDrive
Code for simulations and controllers pertaining to Webots and the project auto_drive

# Vehicle Simulation
Each simulation contains a set of proto nodes and devices that make up a world. These nodes have different properties and functions that can be used alongside them.
proto nodes can be sub divided into Vehicles, Robots, Objects and Devices with dedicated Classes dedicated to them

## Controller class
This is used to control the functionalities of a Node. The controller is a script writeen in c, cpp, python or matlab. In order to implement a controller for a 
node we go to its controller and select the script thast required. we can make a new controller script by new -> new robot controller. We use the kyboard class 
under this. 

## Vehicle class
This is used to avail the functionalities of a vehicle through the driver and car class. The driver class has a set of functions that can control the parameters of a car
such as setting throttle, speed or gear. https://www.cyberbotics.com/doc/automobile/driver-library

## Functioning
### Drag 
We model drag taking into account the rolling resistance and the aerodynamic loads. The drag coeff. and rolling friction coeff are taken from sources and the
drag is computed. Since, there is no proper way to enforce drag, we do so by using the setBrakeIntensity function which is a value between 1 and 0. The braking force offered at BI = 1 is calculated by braking the car from a known speed and taking into account the stopping time, and the avg. decceleration is calculated. The drag forced is then mapped into BI by using this value

### Power 
We take into account the torque rpm characteristics of a PMSM motor, we see that there are two main regions, the constant torque and the contant power region where
torque is inversely proportional to the rpm. this is implemented using the CheckPower function

### Commands 
This is a dictionary which stored the key and the corresponding command ot be sent to the car as a key value pair. The values are in the form of 1x4 arrays, where the
first element refers tot he throttle, second to the steering angle, third to the gear engaged ( in case of an electric car 1/ fwd and -1/ rev ) and the fourth one is boolean representing the 
engagement of brakes.

### Drive
This function thent akes these values and sends the appropraite commands to the car. It is important to note that the setThrottle function works only if a gear is engaged

### Note
The Default torque commands for moving forward and reverse is 0.5 times the max available torque.
The Default steering angle is set at 0.3 degrees, A future model could have an incremental steering command with a restoring steer command too.
The values of the parameters can eb changed easily to suit the code to any car

