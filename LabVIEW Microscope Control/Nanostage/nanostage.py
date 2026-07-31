from ctypes import *
mad = cdll.LoadLibrary(r'.\Madlib.dll')
import json

def pid_controller(pid_json: str, output_range_min, output_range_max, pid_gains, setpoint, process_variable, reinitialise, dt):
    """
    This function implements the core logic of the PID controller. It takes the setpoint, current process variable (PV),
    PID gains (kp, ki, kd), previous error, integral of the error, and the time step (dt) as inputs. It computes the
    current error as the difference between the setpoint and the PV. The integral term accumulates the error over time,
    while the derivative term estimates the rate of change of the error. The control output is calculated by combining
    these three terms, each scaled by their respective gains (kp, ki, kd). The function returns the control output,
    current error, and updated integral.

    Parameters
    ----------
    pid_json : str
        The collection of values that are kept for persistence between functions calls.
    output_range_min, output_range_max : float
        The output range into which outputs are coerced to fit in.
    pid_gains : list[float, float, float]
        The x, y, z values for PID
    setpoint : float
        The target value for the QPD
    process_variable : float
        The current QPD value.
    reinitialise : bool
        Flag for if the system is being reinitialised.
    dt: float
        Time step since last call

    Returns
    -------
    pid_json: str
        JSON with the persistent values needed for next call.
    output: float
        The output pid value
    """
    # Unpack json
    pid_dict = json.loads(pid_json)
    previous_pid_gains = [pid_dict['x'], pid_dict['y'], pid_dict['z']]
    previous_process_variable = pid_dict["process_variable"]
    derivative_action = pid_dict["derivative_action"]
    previous_output = pid_dict["output"]
    previous_error = pid_dict["error"]
    integrated_error = pid_dict["integrated_error"]

    # SET PROCESS VARIABLE
    if dt <= 0 and not reinitialise:
        new_process_variable = previous_process_variable
    else:
        new_process_variable = process_variable

    # SET DERIVATIVE ACTION VARIABLE
    if reinitialise:
        new_derivative_action = 0
    elif dt>0:
        new_derivative_action = -(process_variable - previous_process_variable)*pid_gains[0]*pid_gains[2] / dt
    else:
        new_derivative_action = derivative_action

    # SET ERROR VARIABLES
    error = setpoint - process_variable

    # CALCULATE INTEGRATED ERROR
    skip = False
    ei = 0

    # Check for potential division by zero
    try:
        _ = pid_gains[0]/pid_gains[1]
    except ZeroDivisionError:
        ei = 0
        skip = True

    # If no change from previous call calculate from output value
    if not skip:
        if not previous_pid_gains==pid_gains or reinitialise:
            ei = previous_output -(pid_gains[0]*error + new_derivative_action)
        else:
            # If reinitialising set integrated error to 0
            if reinitialise:
                ei = 0
            else:
                # Else calculate integrated error
                ei = (error+previous_error)*pid_gains[0]*dt/(120*pid_gains[1]) + integrated_error

            # Coerce the value to be inside the output range with consideration for error
            if ei + (error*pid_gains[0]) < output_range_min:
                ei = output_range_min-(error*pid_gains[0])
            elif ei + (error*pid_gains[0]) > output_range_max:
                ei = output_range_max - (error*pid_gains[0])
    integrated_error = ei

    # CALCULATE OUTPUT VALUE
    if skip:
        output = pid_gains[0]*error + new_derivative_action
    elif not previous_pid_gains==pid_gains or reinitialise:
        output = previous_output
    else:
        output = integrated_error + error*pid_gains[0] + new_derivative_action

    # Coerce value to output_range
    if output < output_range_min:
        output = output_range_min
    elif output > output_range_max:
        output = output_range_max
    else:
        output = output

    # Update json values and package
    pid_dict["x"], pid_dict["y"], pid_dict["z"] = pid_gains
    pid_dict["process_variable"] = new_process_variable
    pid_dict["derivative_action"] = new_derivative_action
    pid_dict["output"] = output
    pid_dict["error"] = error
    pid_dict["integrated_error"] = integrated_error

    pid_json = json.dumps(pid_dict)

    return pid_json, output

def read_position(handle_in):
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

def force_clamp(pid_json, desired_force, nanostage, qpd, pos0, handle_in, dt,
                output_range_min, output_range_max, pid_gains, reinitialise, apply_force):
    """

    """
    setpoint = desired_force+pos0
    if apply_force:
        qpd_delta = qpd - pos0

        pid_json, pid_output = pid_controller(pid_json, output_range_min, output_range_max, pid_gains, setpoint, qpd_delta, reinitialise, dt)
        mad.MCL_SingleReadN(nanostage+pid_output, 1, handle_in)
        return pid_json, [qpd_delta, setpoint, pid_output]
    else:
        return pid_json, [0, setpoint, 0]