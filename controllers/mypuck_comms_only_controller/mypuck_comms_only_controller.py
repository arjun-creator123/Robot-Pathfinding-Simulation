"""mypuck_comms_only_controller controller."""

from controller import Robot
import struct
import math
import sys

# 1. INITIALIZE ROBOT & SENSORS
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

camera = robot.getDevice("colour_camera")
camera.enable(timeStep)
gps = robot.getDevice("gps")
gps.enable(timeStep)
compass = robot.getDevice("compass")
compass.enable(timeStep)
emitter = robot.getDevice("emitter")
up_sensor = robot.getDevice("up_sensor")
up_sensor.enable(timeStep)
right_sensor = robot.getDevice("right_sensor")
right_sensor.enable(timeStep)
left_sensor = robot.getDevice("left_sensor")
left_sensor.enable(timeStep)
down_sensor = robot.getDevice("down_sensor")
down_sensor.enable(timeStep)
speaker = robot.getDevice("speaker")

# CRITICAL FIX: Step once to clear NaN sensor values on startup
robot.step(timeStep)

mode = sys.argv[1].upper() if len(sys.argv) > 1 and sys.argv[1].strip() != "" else "DFS"
print(f"🤖 E-puck sensors active for {mode} mode.")

done = False
local_dict = {}

def obstacle_finder(inp):
    if inp >= 0.65: return True
    else: return False

def robot_orientation(compass_val, surrounding, right_value, up_value, left_value, down_value):
    cx, cy, cz = compass_val
    max_axis = max(abs(cx), abs(cy), abs(cz))
    
    if max_axis == abs(cx):
        if cx < 0: surrounding = [obstacle_finder(right_value), obstacle_finder(up_value), obstacle_finder(left_value), obstacle_finder(down_value)]
        else: surrounding = [obstacle_finder(left_value), obstacle_finder(down_value), obstacle_finder(right_value), obstacle_finder(up_value)]
    elif max_axis == abs(cz):
        if cz > 0: surrounding = [obstacle_finder(up_value), obstacle_finder(left_value), obstacle_finder(down_value), obstacle_finder(right_value)]
        else: surrounding = [obstacle_finder(down_value), obstacle_finder(right_value), obstacle_finder(up_value), obstacle_finder(left_value)]
    else:
        if cy > 0: surrounding = [obstacle_finder(right_value), obstacle_finder(up_value), obstacle_finder(left_value), obstacle_finder(down_value)]
        else: surrounding = [obstacle_finder(left_value), obstacle_finder(down_value), obstacle_finder(right_value), obstacle_finder(up_value)]
    return surrounding

# MAIN LOOP
while robot.step(timeStep) != -1:
    timings = range(2,180,1)
    for index, time in enumerate(timings):
        while robot.getTime() < time:
            if robot.step(timeStep) == -1:
                quit()
                
        X_pos = round(gps.getValues()[0],1)
        Z_pos = round(gps.getValues()[2],1)
        
        right_value = right_sensor.getValue()
        up_value = up_sensor.getValue()
        left_value = left_sensor.getValue()
        down_value = down_sensor.getValue()
        
        raw_compass = compass.getValues()
        if math.isnan(raw_compass[0]):
            continue 
            
        compass_val = [round(raw_compass[0],1), round(raw_compass[1],1), round(raw_compass[2],1)]
        
        TARGET_X = 1.5  
        TARGET_Z = 1.5  
        distance = math.hypot(X_pos - TARGET_X, Z_pos - TARGET_Z)
        
        if distance < 0.6:
            done = True
            try: speaker.playSound(speaker, speaker, "target_found.wav", 1.0, 1.0, 0.0, True)
            except: pass
                        
        surrounding = []
        surrounding = robot_orientation(compass_val, surrounding, right_value, up_value, left_value, down_value)
        local_dict[(X_pos,Z_pos)] = local_dict.get((X_pos,Z_pos), surrounding)
        
        message = struct.pack("? f f ? ? ? ?", done, X_pos, Z_pos, surrounding[0], surrounding[1], surrounding[2], surrounding[3])
        emitter.send(message)