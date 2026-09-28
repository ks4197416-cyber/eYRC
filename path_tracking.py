import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

WHEELBASE = 0.120
TRACK_WIDTH = 0.110
WHEEL_OFFSET = 0.0275

VEHICLE_PATH = '/Niti_Vahan'
LEFT_STEER_JOINT = '/steering_left_joint'
RIGHT_STEER_JOINT = '/steering_right_joint'
DRIVE_MOTOR = '/drive_motor'
PROXIMITY_SENSOR = '/proximity_sensor' 

def ackermann_wheel_angles(delta):
    if delta == 0:
        return 0.0, 0.0
    d_eff = TRACK_WIDTH / 2.0 - WHEEL_OFFSET
    tan_delta = math.tan(delta)
    left_angle = math.atan(tan_delta / (1.0 - (d_eff / WHEELBASE) * tan_delta))
    right_angle = math.atan(tan_delta / (1.0 + (d_eff / WHEELBASE) * tan_delta))
    return left_angle, right_angle

def compute_steering(target_y, current_values):
    cross_track_error = current_values['y'] - target_y
    heading_error = -current_values['yaw']
    heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
    
    speed = current_values['speed']
    k = 1.5
    k_s = 0.1
    
    steering = heading_error + math.atan((k * cross_track_error) / (speed + k_s))
    return float(steering)

def run_simulation():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    sim.startSimulation()
    print("Simulation started successfully.")

    vehicle = sim.getObject(VEHICLE_PATH)
    left_steer_handle = sim.getObject(LEFT_STEER_JOINT)
    right_steer_handle = sim.getObject(RIGHT_STEER_JOINT)
    drive_motor_handle = sim.getObject(DRIVE_MOTOR)
    
    current_lane = 0
    lane_y_coords = [0.0, 0.20] 
    target_y = lane_y_coords[current_lane]
    
    obstacle_detected = False
    
    try:
        while True:
            pos = sim.getObjectPosition(vehicle, -1) 
            orientation = sim.getObjectOrientation(vehicle, -1)
            
            current_values = {
                'x': pos[0],
                'y': pos[1],
                'yaw': orientation[2],
                'speed': 0.5 
            }
         
            try:
                sensor_handle = sim.getObject(PROXIMITY_SENSOR)
                res, detected, distance, _, _ = sim.readProximitySensor(sensor_handle)
                
                if res and distance < 0.6:
                    if not obstacle_detected:
                        print(f"Obstacle detected at distance {distance:.2f}m! Changing lane automatically...")
                        current_lane = 1 - current_lane 
                        target_y = lane_y_coords[current_lane]
                        obstacle_detected = True
                else:
                    if distance > 0.8:
                        obstacle_detected = False
            except Exception as e:
                pass

            steering_angle = compute_steering(target_y, current_values)
            left_angle, right_angle = ackermann_wheel_angles(steering_angle)
            
            sim.setJointTargetAngle(left_steer_handle, left_angle)
            sim.setJointTargetAngle(right_steer_handle, right_angle)
            sim.setJointTargetVelocity(drive_motor_handle, 2.0)
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Stopping simulation...")
        sim.stopSimulation()
        print("Simulation stopped.")

if __name__ == '__main__':
    run_simulation()