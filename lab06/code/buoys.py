# Create Library
import createlib as cl
from collections import namedtuple

IRSensors = namedtuple('IRSensors', ['omni','left', 'right'])
IRSensor = namedtuple('IRSensor', ['reserved','force_field', 'green_buoy', 'red_buoy'])

def get_sensors(sensors: cl.Sensors):
    return IRSensors(
        omni=parse_sensor(sensors.ir_opcode),
        left=parse_sensor(sensors.ir_opcode_left),
        right=parse_sensor(sensors.ir_opcode_right),
    )

def parse_sensor(got):
    reserved = False
    red_buoy = False
    green_buoy = False
    force_field = False

    if got >= 160:
        reserved = True
        got = got - 160
    if got >= 8:
        red_buoy = True
        got = got - 8
    if got >= 4:
        green_buoy = True
        got = got - 4
    if got >= 1:
        force_field = True
        got = got - 1

    buoys = IRSensor(
        reserved=reserved,
        force_field=force_field,
        green_buoy=green_buoy,
        red_buoy=red_buoy,
    )

    return buoys
