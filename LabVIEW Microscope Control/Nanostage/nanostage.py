from ctypes import *
mad = cdll.LoadLibrary(r'C:\Users\psmr500\PycharmProjects\COMBI-Tweez\LabVIEW Microscope Control\Nanostage\Madlib.dll')
from pid import pid_controller

def read_positon(handle_in):
    """
    Takes the handle and returns the position using the MADLib.dll, MCL_SingleReadN

    Parameters
    ----------
    handle_in: int
        integer providing addressing info for hardware

    Returns
    -------
    list[int, int, int]:
        x, y, z position readings
    """
    position = [mad.MCL_SingleReadN(x+1, handle_in) for x in range(3)]
    return position

def force_clamp(pid_json, desired_force, nanostage, QPD, pos0, handle_in, dt,
                output_range_min, output_range_max, PID_gains, reinitialise, apply_force):
    """

    """
    setpoint = desired_force+pos0
    if apply_force:
        QPD_delta = QPD - pos0

        pid_json, pid_output = pid_controller(pid_json, output_range_min, output_range_max, PID_gains, setpoint, QPD_delta, reinitialise, dt)
        mad.MCL_SingleReadN(nanostage+pid_output, 1, handle_in)
        return pid_json, QPD_delta, setpoint, pid_output
    else:
        return pid_json, 0, setpoint, 0