import rospy
from typing import NamedTuple


class FlightMode(NamedTuple):
    mode: str
    is_simulation: bool
    is_hardware: bool
    is_real: bool


def get_flight_mode() -> FlightMode:
    try:
        flight_mode = rospy.get_param("flight_mode")
    except KeyError as exc:
        # raise KeyError('rosparam "flight_mode" not set')
        raise exc

    mode = flight_mode
    is_simulation = False
    is_hardware = False
    is_real = False

    if flight_mode == 'simulation':
        is_simulation = True
    elif flight_mode == 'hardware':
        is_hardware = True
    elif flight_mode == 'real':
        is_real = True
    else:
        raise KeyError('rosparam "flight_mode" is an invalid value')

    return FlightMode(mode=mode, is_simulation=is_simulation, is_hardware=is_hardware, is_real=is_real)
