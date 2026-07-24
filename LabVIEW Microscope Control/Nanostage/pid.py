def pid_controller(setpoint: float, pv: float, pid_gains, previous_pid_gains,
                   process_variable, previous_process_variable,
                   derivative_action,
                   previous_error: float, integrated_error: float, dt: float,
                   output_range, reinitialise, previous_output):
    """
    This function implements the core logic of the PID controller. It takes the setpoint, current process variable (PV),
    PID gains (kp, ki, kd), previous error, integral of the error, and the time step (dt) as inputs. It computes the
    current error as the difference between the setpoint and the PV. The integral term accumulates the error over time,
    while the derivative term estimates the rate of change of the error. The control output is calculated by combining
    these three terms, each scaled by their respective gains (kp, ki, kd). The function returns the control output,
    current error, and updated integral.

    Parameters
    ----------

    """
    # Update pid_gains
    new_pid_gains = pid_gains

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
    error = setpoint - pv

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
            if ei + (error*pid_gains[0]) < output_range[0]:
                ei = output_range[0]-(error*pid_gains[0])
            elif ei + (error*pid_gains[0]) > output_range[1]:
                ei = output_range[1] - (error*pid_gains[0])
    integrated_error = ei

    # CALCULATE OUTPUT VALUE
    if skip:
        output = pid_gains[0]*error + new_derivative_action
    elif not previous_pid_gains==pid_gains or reinitialise:
        output = previous_output
    else:
        output = integrated_error + error*pid_gains[0] + new_derivative_action

    # Coerce value to output_range
    if output < output_range[0]:
        output = output_range[0]
    elif output > output_range[1]:
        output = output_range[1]
    else:
        output = output


    return new_pid_gains, new_process_variable, new_derivative_action, error, integrated_error, output





