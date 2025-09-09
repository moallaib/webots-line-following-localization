"""line_following_Localization controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
import numpy as np
from matplotlib import pyplot as plt
# create the Robot ifrom controller import Robot

# Create the Robot instance
robot = Robot()

# Time step of the simulation
timestep = int(robot.getBasicTimeStep())

# Constants specific to e-puck
wheel_radius = 0.021  # Radius of e-puck wheels in meters
wheel_base = 0.053    # Distance between e-puck wheels in meters

# Initialize variables
MAX_SPEED=6.28
xw = 0.0
yw = 0.028
omegaz = 0.0
# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
display=robot.getDevice('display')

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

lidar=robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()


#display=robot.getDevice('display')
# Initialize ground sensors
gs = []
gs_names = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gs_names[i]))
    gs[i].enable(timestep)

# Function to update position and orientation
def update_position(wheel_speed_left, wheel_speed_right, dt):
    global xw, yw, omegaz
    # Calculate linear and angular velocities
    v = wheel_radius * (wheel_speed_left + wheel_speed_right) / 2
    omega = wheel_radius * (wheel_speed_right - wheel_speed_left) / wheel_base
    # Update position and orientation
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta=np.arctan2(compass.getValues()[0],compass.getValues()[1])
    omegaz += omega * dt
# Main loop
def world2map(xw, yw):
    # Assuming the world coordinates range from -150 to 150 for both x and y
    # Map these coordinates to the grid indices (0 to 299)
    px = int((xw + 0.3) * (150 / 0.6))  # Scale x from -0.3 to 0.3 to 0 to 299
    py = abs(int((yw + 0.3) * (150 / 0.6))-300)  # Scale y from -0.22 to 0.22 to 0 to 299
    
    
    return [px, py]
    
angles=np.linspace(3.1415,-3.1415,360)  
while robot.step(timestep) != -1:
 

    # Read ground sensor values
    g = [gs[i].getValue() for i in range(3)]
    
    if (g[0] > 500 and g[1]<350 and g[2]>500): # drive straight
        wheel_speed_left , wheel_speed_right  = MAX_SPEED, MAX_SPEED
    elif(g[2]<550): # turn right
        wheel_speed_left, wheel_speed_right = 0.25 * MAX_SPEED, -0.109*MAX_SPEED
    elif(g[0]<550):  # turn left
        wheel_speed_left, wheel_speed_right = -0.109 * MAX_SPEED, 0.25*MAX_SPEED
    else:
        wheel_speed_left, wheel_speed_right = 0,0
    dt = timestep / 1000.0  # Convert timestep to seconds
    #update_position(wheel_speed_left, wheel_speed_right, dt)
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    w_T_r = np.array([[np.cos(theta),-np.sin(theta), xw],
                  [np.sin(theta),np.cos(theta), yw],
                  [0,0,1]])            
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    X_r = np.array([ranges*np.cos(angles), 
                ranges*np.sin(angles),
                np.ones(len(angles))])
    D = w_T_r @ X_r
    
    px, py = world2map(xw, yw)
    display.setColor(0xFF0000)  # Set ink color to red
    display.drawPixel(px,py)   # Draw the pixel on the display
    display.setColor(0xFFFFFF)  # Set ink color to white
    for i in range(D.shape[1]):
        px, py = world2map(D[0, i], D[1, i])
        display.drawPixel(px, py)  # Draw each laser scan point on the display
    left_motor.setVelocity(wheel_speed_left)
    right_motor.setVelocity(wheel_speed_right)
    # Printing results
    print(f"px: {px}, py: {py}, omegaz: {omegaz}")
    print(f"Error: {np.sqrt(xw**2 + yw**2)}")
    
    
