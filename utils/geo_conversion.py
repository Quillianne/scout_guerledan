import numpy as np
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils.settings import POINT_BASE, RHO

def convert_to_decimal_degrees(ddmmss, direction):
    """
    Converts a latitude/longitude in the DDMM.MMMM format 
    (degrees + decimal minutes) to decimal degrees.

    Parameters
    ----------
    ddmmss : float or str
        The latitude/longitude value in DDMM.MMMM format.
        For example, 4807.038 indicates 48 degrees and 07.038 minutes.
    direction : str
        The direction, 'N', 'S', 'E' or 'W'. 
        If direction is 'S' or 'W', the value will be negative.

    Returns
    -------
    float
        The latitude/longitude in decimal degrees.
    """
    # Convert to float if the value is passed as a string
    ddmmss = float(ddmmss)

    # Separate the degrees and minutes
    degrees = int(ddmmss // 100)   # Integer part corresponding to the degrees
    minutes = ddmmss % 100         # Remainder corresponding to the minutes

    # Convert minutes to decimal degrees
    decimal_degrees = degrees + (minutes / 60.0)

    # Handle direction (South or West => negative value)
    if direction.upper() in ['S', 'W']:
        decimal_degrees = -decimal_degrees

    return decimal_degrees

def deg_to_rad(deg):
    """Converts degrees to radians."""
    return deg * np.pi / 180

def rad_to_deg(rad):
    """Converts radians to degrees."""
    return rad * 180.0 / np.pi

def conversion_cartesien_spherique(coord_xy, lat_m=POINT_BASE[0], long_m=POINT_BASE[1], rho=RHO):
    """
    Inverse of conversion_spherique_cartesien:
    From local coordinates (X, Y), returns (latitude, longitude) in degrees 
    for the corresponding point.
    
    Parameters
    ----------
    coord_xy : tuple
        Tuple (X, Y) in local coordinates.
    lat_m, long_m : float
        Coordinates of the reference point (in degrees).
    rho : float
        Radius (or major radius) of the sphere used.
    
    Returns
    -------
    tuple
        (latitude, longitude) in degrees of point P.
    """

    X, Y = coord_xy

    # Convert the reference (lat_m, long_m) to radians
    lat_m_rad = deg_to_rad(lat_m)
    long_m_rad = deg_to_rad(long_m)

    # Cartesian coordinates (x_m, y_m) of point M
    x_m = rho * np.cos(lat_m_rad) * np.cos(long_m_rad)
    y_m = rho * np.cos(lat_m_rad) * np.sin(long_m_rad)

    # Retrieve (x_p, y_p) of point P
    # Reminder: X = x_m - x_p ; Y = y_p - y_m
    x_p = x_m - X
    y_p = y_m + Y

    # Compute the longitude (radians)
    long_p_rad = np.arctan2(y_p, x_p)

    # Compute the latitude (radians) via cos(lat_p)
    # cos(lat_p) = sqrt(x_p^2 + y_p^2) / rho
    r_xy = np.sqrt(x_p**2 + y_p**2)
    cos_lat_p = r_xy / rho

    # Clamp cos_lat_p to 1 (in case of numerical imprecisions >1)
    if cos_lat_p > 1.0:
        cos_lat_p = 1.0
    elif cos_lat_p < -1.0:
        cos_lat_p = -1.0

    lat_p_rad = np.arccos(cos_lat_p)
    
    # Choose the sign for lat_p:
    # Hypothesis: remain in the same hemisphere as lat_m.
    # If lat_m is negative, then lat_p is made negative, etc.
    if lat_m_rad < 0:
        lat_p_rad = -lat_p_rad

    # Conversion to degrees
    lat_p_deg  = rad_to_deg(lat_p_rad)
    long_p_deg = rad_to_deg(long_p_rad)

    return (lat_p_deg, long_p_deg)

def conversion_spherique_cartesien(point, lat_m=POINT_BASE[0], long_m=POINT_BASE[1], rho=RHO):
    """
    Converts GPS coordinates (latitude, longitude) to local cartesian coordinates
    with respect to a point M defined by lat_m and long_m, returning only x and y.
    
    Parameters
    ----------
    point : tuple
        Tuple (latitude, longitude) in degrees.
    lat_m, long_m : float
        Coordinates of the reference point (in degrees).
    rho : float
        Radius (or major radius) of the sphere used.
    
    Returns
    -------
    tuple
        Local cartesian coordinates (x, y).
    """
    # Convert latitudes and longitudes to radians
    lat_m_rad = deg_to_rad(lat_m)
    long_m_rad = deg_to_rad(long_m)
    lat_rad = deg_to_rad(point[0])
    long_rad = deg_to_rad(point[1])

    # Conversion of reference point M to 2D cartesian coordinates (x_m, y_m)
    x_m = rho * np.cos(lat_m_rad) * np.cos(long_m_rad)
    y_m = rho * np.cos(lat_m_rad) * np.sin(long_m_rad)

    # Conversion of point P to 2D cartesian coordinates (x_p, y_p)
    x_p = rho * np.cos(lat_rad) * np.cos(long_rad)
    y_p = rho * np.cos(lat_rad) * np.sin(long_rad)

    # Compute relative coordinates with respect to point M
    x = x_p - x_m
    y = y_p - y_m

    return -x, y

def gpx_to_cartesian(gpx_file):
    """
    Converts a GPX file to a .npy file.

    Parameters
    ----------
    gpx_file : str
        The name of the input GPX file.
    
    Returns
    -------
    list
        A list of cartesian coordinates (x, y).
    """
    gpx = open(gpx_file, "r", encoding="utf-8")
    coords = []
    for line in gpx:
        if "<trkpt" in line:
            lat = float(line.split('lat="')[1].split('" lon=')[0])
            lon = float(line.split('lon="')[1].split('"></trkpt>')[0])
            x, y = conversion_spherique_cartesien((lat, lon))
            coords.append((x, y))
    
    return coords