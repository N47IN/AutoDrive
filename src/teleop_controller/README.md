### Stepping through some essential blocks and functions used for controlling the car using teleoperation
driver = Driver()
We will be using the Driver library from the controller class, this gives us complete control over all the functionalities of a driver.
The selected car is a Tesla Model S for simulating an EV, drag is applied.

### Using the keyboard
keyboard.getKey(), keyboard.enable(<sampling frequency>), keyboard = Keyboard()
Values returned from getKey are ASCII decimal values.
use the ord(<>) function to check if the value of the pressed key matches with ord of any of WASD and " "

### Moving the car
Heads up - all speeds used in the getCurrentSpeed and setCruisingSpeed functions are in Kmph
#### Gears 
driver.setGear(<-1,0,1>)
The first step to make a car move is to set a gear. The details about the number of gears available for a car can be obtained from webots documentation.
The Tesla has 3 Gears, 1 for forward, 0 for park and -1 for reversing

#### Throttle ( W )
driver.setThrottle(<0-1>), driver.setGear(1)
This is the torque output produced by the Motor, a value between 0-1. 1 would mean max torque and 0.5 would mean half of the max available torque
For the Tesla, Throttle of 1 corresponded to a longitudinal acceleration of 5.02 m/s^2, used for mapping.

#### Deceleration ( " " for Braking, no key for Coasting down due to drag )
driver.setBrakeIntensity(<0-1>)
This is proprotional to the amount of braking force applied, a value between 0-1, a value of 1 would mean that the brakes are fully engaged.
For the Tesla, Brake Intensity of 1 corresponded to a longitudinal deceleration of 7.12 m/s2, should be mapped accordingly.

#### Reversing ( S )
driver.setThrottle(<0-1>), driver.setGear(-1)

#### Steering ( A / D )
driver.setSteeringAngle(<>)
Negative for turning left and Positive for turning right, in radians

#### Commands
commands are given in the form of a dictionary with W, A, S, D being the KEYS and the value being the command given to the car.
the command value pair is a 1x4 array with
1st element indication the torque applied - 0 to 1
2nd elemnt indicating the steering angle 
3rd element indicating the gear to be engaged - -1 / 0 / 1
4th element being a boolean used to check if brakes are engaged or not

Navin Sriram, n47insriram@gmail.com



