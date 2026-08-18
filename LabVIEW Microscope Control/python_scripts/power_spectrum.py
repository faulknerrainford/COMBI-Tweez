import nidaqmx as daqmx
from nidaqmx.constants import TerminalConfiguration, AcquisitionType
import numpy as np
from nitypes.waveform import AnalogWaveform
from Trap_calibration import psd_fit

PHYSICAL_CHANNELS = ("cDAQ1Mod1/ai0", "cDAQ1Mod1/ai1", "cDAQ1Mod1/ai2", "cDAQ1Mod2/ai3", "cDAQ1Mod3/ai0",
                     "cDAQ1Mod3/ai1", "cDAQ1Mod3/ai2", "cDAQ1Mod3/ai3", "cDAQ1Mod1/ai3")
CHANNEL_NAMES = ("LR", "BT", "SUM", "BIAS", "MT_vertical", "MT_horizontal",
                 "nanostage_vertical", "nanostage_horizontal",
                 "camera_fire")
#  TODO: keep case in labview and run python if true but turn off the button in labview, stop and clear task there as well (just to avoid passing a parameter)

def power_spectrum(save:bool, duration:int, loop_length:int, z:float):

    # DAQ finite - create task, create channels (add_ai_voltage_channel), timing (Timing.samp_clk_src)
    task = daqmx.Task()
    task = create_channels(task)
    task.timing.cfg_samp_clk_timing(50000, sample_mode=AcquisitionType.CONTINUOUS, samps_per_chan=100000)
    # start DAQ task (task.start)
    task.start()
    for _ in range(loop_length):
        # read (NChan NSamp Duration)
        samples: list[AnalogWaveform] = task.read_waveform(duration)
        # process signal
        sum_temp = -1*samples[2].t0
        channel_samples = {'SUM':sum_temp,
                           'LR': samples[0].t0/sum_temp,
                           'BT': samples[1].t0/sum_temp,
                           'MT': np.concat(samples[4].t0, samples[5].t0)}
        if save:
            # TODO: ** Save psi** (look into save format)
            # get dt from waveforms
            rate = 1/samples[0].dt
            height = z*1000
        # TODO: run psd3 on x y and z (accumulate as lists)
        for data, axis in zip((channel_samples['LR'], channel_samples['BT'], channel_samples['SUM']),('x','y','z')):
            fp = psd_fit.psd(data)
            fp_fit = psd_fit.fit(fp[0], fp[1], axis)
            # TODO: work out what is going on with this data
    # TODO: get means of lists (see LabVIEW for details)
    # TODO: append and convert data then save to file

def create_channels(task, physical_channels: tuple[str, ...]=PHYSICAL_CHANNELS,
                    channel_names: tuple[str, ...]=CHANNEL_NAMES,
                    min_value: float = -10, max_value: float= 10):
    """
    Creates ai channels to record. uses preset channels, names and scale.
    Currently min value -10 to max value 10

    Parameters
    ----------
    task : nidaqmx.Task
        Task to attach the channels to
    physical_channels : tuple[str, ...]
        list of physical channels to attach the channels to (defaults to PHYSICAL_CHANNELS)
    channel_names : tuple[str, ...]
        list of channel names to attach the channels to (defaults to CHANNEL_NAMES)
    min_value : float
        minimum value to attach the channels to (defaults to -10)
    max_value : float
        maximum value to attach the channels to (defaults to 10)

    Returns
    -------
    task : nidaqmx.Task
        task with attached ai channels

    Raises
    ------
    ValueError
        physical_channels and channel_names must have the same length
    """
    if len(physical_channels) != len(channel_names):
        raise ValueError("physical_channels and channel_names must have the same length")
    terminal_config = TerminalConfiguration.DIFF
    for physical_channel, channel_name in zip(physical_channels, channel_names):
        task.ai_channels.add_ai_voltage_chan(physical_channel, channel_name, terminal_config, min_value, max_value)
    return task