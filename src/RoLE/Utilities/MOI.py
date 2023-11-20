# Typing (Support for type hints)
import typing as tp

def Cube(m: float, Size: tp.List[float]) -> tp.Tuple[float]:
    """
    Description:
        Obtain the moment of inertia (MOI) for the bounding box (cuboid).

    Args:
        (1) m [float]: The mass of the box.
        (2) Size [Vector<float> 1x3]: The size of the box in three-dimensional space (x, y, z).

    Retruns:
        (1) paramterer [Dictionary{'I_xx': float, 'I_xy': float, etc.}]: The moment of inertia (MOI) of the bounding 
                                                                         box (cuboid).
    """
    
    # Abbreviations.
    division_part = m * 0.0833
    x_square = Size[0] ** 2; y_square = Size[1] ** 2
    z_square = Size[2] ** 2

    return {'I_xx': division_part * (y_square + z_square), 
            'I_xy': 0.0, 
            'I_xz': 0.0,
            'I_yy': division_part * (x_square + z_square), 
            'I_yz': 0.0, 
            'I_zz': division_part * (x_square + y_square)}