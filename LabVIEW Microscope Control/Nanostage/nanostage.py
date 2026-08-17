from __future__ import annotations
from ctypes import *

mad = cdll.LoadLibrary(r'.\Madlib.dll')
import json, math, time

def pid_controller(pid_json: str, output_range_min, output_range_max, pid_gains, setpoint, process_variable, reinitialise, dt)-> tuple[str, float]:
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

def force_clamp(pid_json: str, desired_force, nanostage, qpd, pos0, handle_in, dt,
                output_range_min, output_range_max, pid_gains, reinitialise, apply_force):
    """
    Operates the forces clamp and runs pid controller.

    Parameters
    ----------
    pid_json: string
        JSON file of persistent values for the pid controller
    desired_force: float
        desired force on clamp
    nanostage: int
    qpd: float
    pos0: float
        Initial position of the clamp
    handle_in: int
        integer providing addressing info for hardware
    dt: float
        Time interval since previous call
    output_range_min: float
        Min spectrum values, values are coerced to be greater than or equal to this.
    output_range_max: float
        Max spectrum values, values are coerced to be less than or equal to this.
    pid_gains: list[float, float, float]
        x,y,z position readings
    reinitialise: bool
        flag to indicate if pid controller is being reset
    apply_force: bool
        flag to indicate if force clamp should be applied

    Returns
    -------
    string
        JSON of persistent values for the pid controller
    list[float, float, float]:
        qpd_delta: float
        setpoint: float
        pid_output: float
    """
    setpoint = desired_force+pos0
    if apply_force:
        qpd_delta = qpd - pos0
        pid_json, pid_output = pid_controller(pid_json, output_range_min, output_range_max, pid_gains, setpoint, qpd_delta, reinitialise, dt)
        mad.MCL_SingleReadN(nanostage+pid_output, 1, handle_in)
        return pid_json, [qpd_delta, setpoint, pid_output]
    else:
        return pid_json, [0, setpoint, 0]

def interval_move(axis: int, handle: int, destination: float, speed: float, interval: float)->float:
    """
    Moves nanostage in a single axis to a given position and returns the new position.

    Parameters
    ----------
    axis: int
        axis of movement to position nanostage in.
    handle: int
        integer providing addressing info for hardware
    destination: float
        requested position
    speed: float
        um/s speed for how quick to move the nanostage
    interval: float
        number of moves to use to move the nanostage

    Return
    ------
    float:
        new position of the nanostage
    """
    read_return = mad.MCL_SingleReadN(axis, handle)
    index = math.ceil(abs((destination-read_return)/(speed*interval)))
    wait = interval * 1000
    if destination-read_return>0:
        speed_interval = speed*interval
    else:
        speed_interval = -(speed*interval)
    for i in range(index):
        mad.MCL_SingleWriteN(i*speed_interval+read_return, axis, handle)
        time.sleep(wait)
    mad.MCL_SingleReadN(destination, axis, handle)
    position = mad.MCL_SingleReadN(axis, handle)
    return position

def centre_stage(destination, handle):
    """
    Moves nanostage to a given position.

    Parameters
    ----------
    destination: float
        desired position
    handle: int
        integer providing addressing info for hardware
    """
    for i in range(3):
        interval_move(i+1, handle, destination, 100, 0.05)
    return handle

def nanostage_moving(handle: int, step_size: bool, fine: float, course: float, keyboard_input: str,
                     left: bool, right: bool, forward: bool, back: bool, up: bool, down: bool,
                     x: float, y: float, z: float):
    """
    Allows the nanostage to be repositioned based on keyboard and interface input.

    Parameters
    ----------
    handle: int
        integer providing addressing info for hardware
    step_size: bool
        Sets movement in response to input as either fine or course
    fine: float
        Sets the smaller step size for nanostage movement
    course: float
        Sets the larger step size for nanostage movement
    keyboard_input: str
        Provides a string value representing the keyboard input
    left: bool
        Indicates if the left button in the interface is being pressed
    right: bool
        Indicates if the right button in the interface is being pressed
    forward: bool
        Indicates if the forward button in the interface is being pressed
    back: bool
        Indicates if the backward button in the interface is being pressed
    up: bool
        Indicates if the up button in the interface is being pressed
    down: bool
        Indicates if the down button in the interface is being pressed
    x: float
        Position in the x-(0-)axis
    y: float
        Position in the y-(1-)axis
    z: float
        Position in the z-(2-)axis
    """
    if step_size:
        set_step_size = fine/1000
    else:
        set_step_size = course/1000

    # Movement in x axis
    if keyboard_input=='D'or keyboard_input=='\\' or left or right:
        if keyboard_input=='\\' or right:
            new_position = x-set_step_size
        else:
            new_position = x+set_step_size
        mad.MCL_SingleWriteN( new_position, 2, handle)

    # Movement in y-axis
    if keyboard_input=='f' or keyboard_input==';' or forward or back:
        if keyboard_input==';' or down:
            new_position = y-set_step_size
        else:
            new_position = y+set_step_size
        mad.MCL_SingleWriteN( new_position, 1, handle)

    # Movement in z axis
    if keyboard_input=='Cf' or keyboard_input==';C' or up or down:
        if keyboard_input=='Cf' or down:
            new_position = z-set_step_size
        else:
            new_position = z+set_step_size
        mad.MCL_SingleWriteN( new_position, 3, handle)

def exramp_triangle_generator(start: float, end: float, no_positions: int)->list[float]:
    """

    Parameters
    ----------
    start: float
    end: float
    no_positions: int

    Returns
    -------
    List[float]
    """
    step_size = end-start/-no_positions
    array: list[float] = [0 for _ in range(2*no_positions)]
    for i in range(no_positions):
        array[i] = start+(i*step_size)
        array[i+no_positions] = end - (i*step_size)
    return array

def nanostage_tapping(current_x: float, current_y:float, current_z: float,
                      start_x: float, start_y: float, start_z: float, samples: int,
                      amp_x: float, amp_y:float, amp_z: float, handle: int, delay: int):
    if amp_x == 0:
        array_x: list[float] = [start_x for _ in range(2*samples)]
    else:
        array_x = exramp_triangle_generator(start_x, current_x-amp_x, samples)
    if amp_y == 0:
        array_y: list[float] = [start_y for _ in range(2 * samples)]
    else:
        array_y = exramp_triangle_generator(start_y, current_y-amp_y, samples)
    if amp_z == 0:
        array_z: list[float] = [start_z for _ in range(2 * samples)]
    else:
        array_z = exramp_triangle_generator(start_z, current_z-amp_z, samples)

    # Ensure assignment
    x, y, z = current_x, current_y, current_z

    # Movement loop
    for i in range(samples*2):
        # Move wait then read
        # TODO: check if delays need to be between all moves or just between move and read
        mad.MCL_SingleWriteN( array_x[i], 2, handle)
        mad.MCL_SingleWriteN( array_y[i], 1, handle)
        mad.MCL_SingleWriteN( array_z[i], 3, handle)
        time.sleep(delay)
        x = mad.MCL_SingleReadN( 2, handle)
        y = mad.MCL_SingleReadN( 1, handle)
        z = mad.MCL_SingleReadN( 3, handle)

    return [x, y, z]

