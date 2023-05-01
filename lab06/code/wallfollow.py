# Create Library
import createlib as cl

# each sensor range is a tuple: (range_start, range_stop)
light_set_points = {
    # The "right" sensors are not used since our robot follows using its left side
    # 'right': (0, 1200),
    # 'front_right': (0, 1200),
    # 'center_right': (0, 1200),

    # The left sensors have a "happy" range designed to keep the robot driving parallel
    # Once any sensor is outside its "happy" range, the robot will start adjusting
    
    # If this gets too high it means the robot is head on and needs to turn
    # The low of the range is 0, which happens when the robot is going straight
    'center_left': 0,

    # If this gets too high it means the robot is head on and needs to turn
    # The low of the range is very low, which happens when the robot is going straight
    'front_left': 0,
    
    # If this gets too high it means the robot is too close to the wall and needs to adjust away
    # If this gets too low it means the robot is moving away from a convex corner,
    #   which means the robot needs to turn towards the corner
    'left': 35
}

def error(sensors: cl.Sensors):
    left_e = sensors.light_bumper_left - light_set_points["left"]
    front_left_e = sensors.light_bumper_front_left - light_set_points["front_left"]
    center_left_e = sensors.light_bumper_center_left - light_set_points["center_left"]

    total_e = left_e + front_left_e + (center_left_e * 0.75)

    print(
        f"Error = {left_e:5} {front_left_e:5} {center_left_e:5} {0:5} {0:5} {0:5} = {total_e:5}"
    )

    return total_e
