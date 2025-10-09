"""
Utility functions for Dynamixel motors
"""

def position_to_radians(position, resolution=4096):
    """
    Convert Dynamixel position value to radians
    
    Args:
        position (int): Position value from Dynamixel
        resolution (int): Motor resolution (default: 4096 for most models)
    
    Returns:
        float: Position in radians
    """
    return (position / resolution) * 2 * 3.14159265359


def radians_to_position(radians, resolution=4096):
    """
    Convert radians to Dynamixel position value
    
    Args:
        radians (float): Position in radians
        resolution (int): Motor resolution (default: 4096 for most models)
    
    Returns:
        int: Position value for Dynamixel
    """
    return int((radians / (2 * 3.14159265359)) * resolution)


def position_to_degrees(position, resolution=4096):
    """
    Convert Dynamixel position value to degrees
    
    Args:
        position (int): Position value from Dynamixel
        resolution (int): Motor resolution (default: 4096 for most models)
    
    Returns:
        float: Position in degrees
    """
    return (position / resolution) * 360.0


def degrees_to_position(degrees, resolution=4096):
    """
    Convert degrees to Dynamixel position value
    
    Args:
        degrees (float): Position in degrees
        resolution (int): Motor resolution (default: 4096 for most models)
    
    Returns:
        int: Position value for Dynamixel
    """
    return int((degrees / 360.0) * resolution)


def clamp(value, min_value, max_value):
    """
    Clamp a value between min and max
    
    Args:
        value: Value to clamp
        min_value: Minimum allowed value
        max_value: Maximum allowed value
    
    Returns:
        Clamped value
    """
    return max(min_value, min(value, max_value))
