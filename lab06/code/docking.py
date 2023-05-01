# Create Library
import createlib as cl
from collections import namedtuple

IRSensors = namedtuple('IRSensors', ['omni','left', 'right'])
OpcodeDock600 = namedtuple('OpcodeDock600', ['value', 'reserved','force_field', 'green_buoy', 'red_buoy'])

def error(sensors: cl.Sensors):
    ir = get_dock600_opcodes(sensors=sensors)

    error = 0

    print(30*"-"+"\nCalculating error")

    if ir.left.green_buoy:
        print("Left sees green")
        error = error + 1

    if ir.left.red_buoy:
        print("Left sees red")
        error = error + -2

    if ir.right.green_buoy:
        print("Right sees green")
        error = error + 2
    
    if ir.right.red_buoy:
        print("Right sees red")
        error = error + -1

    if ir.omni.green_buoy:
        print("Omni sees green")
        error = error + 3
    
    if ir.omni.red_buoy:
        print("Omni sees red")
        error = error + -3

    return error

def get_dock600_opcodes(sensors: cl.Sensors):
    return IRSensors(
        omni=parse_dock600_opcode(sensors.ir_opcode),
        left=parse_dock600_opcode(sensors.ir_opcode_left),
        right=parse_dock600_opcode(sensors.ir_opcode_right),
    )

def parse_dock600_opcode(got):
    value = got
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

    buoys = OpcodeDock600(
        value=value,
        reserved=reserved,
        force_field=force_field,
        green_buoy=green_buoy,
        red_buoy=red_buoy,
    )

    return buoys

def prettyPrint(ir: IRSensors):
    print(30*"-")
    print(f"Omni: {ir.omni.value}\n\tff={ir.omni.force_field}\n\tgreen={ir.omni.green_buoy}\n\tred={ir.omni.red_buoy}\n\treserved={ir.omni.reserved}")
    print(f"Left: {ir.left.value}\n\tff={ir.left.force_field}\n\tgreen={ir.left.green_buoy}\n\tred={ir.left.red_buoy}\n\treserved={ir.left.reserved}")
    print(f"Right: {ir.right.value}\n\tff={ir.right.force_field}\n\tgreen={ir.right.green_buoy}\n\tred={ir.right.red_buoy}\n\treserved={ir.right.reserved}")
    print(30*"-")