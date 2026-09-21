import nidaqmx as daqmx
from nidaqmx.constants import TerminalConfiguration, AcquisitionType
import numpy as np
import pandas as pd
from nitypes.waveform import AnalogWaveform
from Trap_calibration import psd_fit
import datetime

PHYSICAL_CHANNELS = ("cDAQ1Mod1/ai0", "cDAQ1Mod1/ai1", "cDAQ1Mod1/ai2", "cDAQ1Mod2/ai3", "cDAQ1Mod3/ai0",
                     "cDAQ1Mod3/ai1", "cDAQ1Mod3/ai2", "cDAQ1Mod3/ai3", "cDAQ1Mod1/ai3")
CHANNEL_NAMES = ("LR", "BT", "SUM", "BIAS", "MT_vertical", "MT_horizontal",
                 "nanostage_vertical", "nanostage_horizontal",
                 "camera_fire")
FILEPATH = 'C:\\Users\\localadmin\\Desktop\\Data\\'
COLUM_HEADERS_FULL = ("X_Value", "LR", "BT", "SUM", "BIAS", "MT_vertical", "MT_horizontal", "nanostage_vertical", "nanostage_horizontal", "camera_fire", "comment")


def power_spectrum(save:bool, duration:int, loop_length:int, z:float, filepath:str =FILEPATH, comment: str|None = None):

    # DAQ finite - create task, create channels (add_ai_voltage_channel), timing (Timing.samp_clk_src)
    task = daqmx.Task()
    task = create_channels(task)
    task.timing.cfg_samp_clk_timing(50000, sample_mode=AcquisitionType.CONTINUOUS, samps_per_chan=100000)
    # start DAQ task (task.start)
    task.start()
    # Create empty list to store data
    df_f = pd.DataFrame(columns=("iteration", "x", "y", "z"))

    for iteration in range(loop_length):
        # read (NChan NSamp Duration)
        samples: list[AnalogWaveform] = task.read_waveform(duration)
        # process signal
        sum_temp = -1*samples[2].scaled_data
        channel_samples = {'SUM':sum_temp,
                           'LR': samples[0].scaled_data/sum_temp,
                           'BT': samples[1].scaled_data/sum_temp,
                           'MT_vertical': samples[4].scaled_data,
                           'MT_horizontal': samples[5].scaled_data}
        if save:
            # ** Save psi** (look into save format)
            # get dt from waveforms
            rate = 1/samples[0].timing.sample_interval.total_seconds()
            height = z*1000
            time = datetime.datetime.now()
            timestamp= (str(time.year) + str(time.month) + str(time.day) +
                        "_" + str(time.hour) + str(time.minute) + str(time.second))
            filename = filepath + "spectrum_"+str(rate)+"Hz_h="+str(height)+"ns"+ timestamp+".lvm"
            # save as dataframe with header
            df = pd.DataFrame(channel_samples)
            df.to_csv(filename, sep='\t', index=False)

        # run psd3 on x y and z (accumulate as lists)
        f_values = [iteration]
        for data, axis in zip((channel_samples['LR'], channel_samples['BT'], channel_samples['SUM']),('x','y','z')):
            fp = psd_fit.psd(data)
            fp_fit = psd_fit.fit(fp[0], fp[1], axis)
            # work out what is going on with this data
            f_values.append(fp_fit[0])
        df_f.iloc[len(df_f)] = f_values

    # mean of x, y, z values for each iteration
    df_means = df_f.groupby("iteration")[["x", "y", "z"]].mean().reset_index()
    # save to tsv file with headers
    df_means.to_csv(filepath + "mean_spectrum.tsv", sep='\t', index=False)
    # clean up DAQ task
    task.stop()
    task.close()


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


def waveforms_to_lvm(waveforms):
    """
    Convert a list of analogue waveforms into a pandas DataFrame and LVM text format.

    Extracts scaled signal data and timing from a list of waveforms and reformats them to LabVIEW (.lvm) structure.

    Parameters
    ----------
    waveforms : list of AnalogWaveform
        A list of waveform objects. Each waveform must contain signal data
        (via ``scaled_data``) and timing properties (via ``timing`` and
        ``sample_count``).

    Returns
    -------
    df : pandas.DataFrame
        A DataFrame containing paired time (`X_Value_i`) and signal data
        columns for each input waveform, padded with ``numpy.nan`` if signal
        lengths vary.
    lvm_text : str
        A tab-separated string including the standard LabVIEW Measurement header
        metadata section (e.g., channels, sample counts, start dates, and time
        intervals) followed by the tab-delimited signal data.

    """

    # 1. Determine common row count
    max_samples = max(wfm.sample_count for wfm in waveforms)

    # 2. Extract Data and Time columns for each signal
    columns_data = {}

    for i, wfm in enumerate(waveforms):
        sig_name = wfm.channel_name or f"Signal_{i}"

        # Determine time delta (dt)
        if wfm.timing.has_sample_interval:
            dt = wfm.timing.sample_interval.total_seconds()
        else:
            dt = 1.0  # Default fallback if timing is empty

        # Build relative time array for this channel
        time_array = np.arange(wfm.sample_count) * dt
        data_array = wfm.scaled_data

        # Pad short arrays with NaN if waveforms have unequal lengths
        if len(time_array) < max_samples:
            pad_len = max_samples - len(time_array)
            time_array = np.pad(time_array, (0, pad_len), constant_values=np.nan)
            data_array = np.pad(data_array, (0, pad_len), constant_values=np.nan)

        # LVM wide format: [X_Channel, Y_Channel]
        columns_data[f"X_Value_{i}"] = time_array
        columns_data[sig_name] = data_array

    # 3. Create DataFrame matching LVM signal layout
    df = pd.DataFrame(columns_data)

    # 4. Generate Tab-Separated string with LabVIEW-style Header
    header_lines = [
        "LabVIEW Measurement",
        "Writer_Version\t0.9",
        "Reader_Version\t0.9",
        "Separator\tTab",
        "Decimal_Separator\t.",
        "***End_Header***",
        "",
        "Channels\t" + "\t".join(str(i + 1) for i in range(len(waveforms))),
        "Samples\t" + "\t".join(str(wfm.sample_count) for wfm in waveforms),
        "Date\t" + "\t".join(
            wfm.timing.start_time.strftime("%Y/%m/%d") if wfm.timing.has_timestamp else "1900/01/01"
            for wfm in waveforms
        ),
        "Time\t" + "\t".join(
            wfm.timing.start_time.strftime("%H:%M:%S.%f") if wfm.timing.has_timestamp else "00:00:00.000"
            for wfm in waveforms
        ),
        "X_Dimension\t" + "\t".join(["Time"] * len(waveforms)),
        "X0\t" + "\t".join(["0.0000000000000000E+0"] * len(waveforms)),
        "Delta_X\t" + "\t".join(
            f"{wfm.timing.sample_interval.total_seconds():.16E}" if wfm.timing.has_sample_interval else "1.0E+0"
            for wfm in waveforms
        ),
        "***End_Header***",
    ]

    # Export DataFrame to tab-separated text
    data_tsv = df.to_csv(sep="\t", index=False)
    lvm_text = "\n".join(header_lines) + "\n" + data_tsv

    return df, lvm_text