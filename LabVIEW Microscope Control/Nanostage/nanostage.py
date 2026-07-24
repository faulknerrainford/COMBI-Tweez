import ctypes
from ctypes import *
mad = cdll.LoadLibrary(r'C:\Users\psmr500\PycharmProjects\COMBI-Tweez\LabVIEW Microscope Control\Nanostage\Madlib.dll')

def read_positon(handle_in):
    """
    Takes the handle and returns the position using the MADLib.dll, MCL_SingleReadN

    Parameters
    ----------
    handle_in: int
        integer providing addressing info for hardware

    Returns
    -------
    x: float
        Position in the x axis
    y: float
        Position in the y axis
    z: float
        Position in the z axis
    """
    position = [mad.MCL_SingleReadN(x+1, handle_in) for x in range(3)]
    return position[0], position[1], position[2]