from calibration_matrices import calibration_matrix


def get_boundary_indexes(value, axis):
    """
    Finds indexes that bound a given value on a given axis in a calibration matrix

    Args:
        value (float): x or y of a point we want to calculate boundary indexes
        axis (int): 0 for x-axis and 1 for y-axis

    Returns:
        tuple (int, int): indexes that bound a given value on a given axis in a calibration matrix
    """

    i1, i2 = 0, 0

    if value >= calibration_matrix[-1][-1][axis]:
        if axis == 0:
            i1 = i2 = len(calibration_matrix) - 1
        else:
            i1 = i2 = len(calibration_matrix[0]) - 1
    elif value > calibration_matrix[0][0][axis]:
        min_value = calibration_matrix[0][0][axis]
        step = calibration_matrix[1 - axis][0 + axis][axis] - min_value
        i1 = int((value - min_value) / step)
        i2 = i1 + 1

    return i1, i2


def get_middle_z(value_start, value_end, value, z_start, z_end):
    """
    Calculates z of a point lying between the start point and end point based on a ratio of values (value is x or y) of
    these points and their z.

    Args:
        value_start (float): x or y of a start point in sim
        value_end (float): x or y of an end point in sim
        value (float): x or y of a point between start point and end point in sim
        z_start (float): z of a start point in sim
        z_end (float): z of an end point in sim

    Returns:
        float: z value of a point in sim
    """

    z_gap = z_end - z_start
    value_gap = value_end - value_start
    value_diff_from_start = value - value_start

    if value_gap == 0:
        ratio = 0
    else:
        ratio = value_diff_from_start / value_gap

    return z_start + ratio * z_gap


def calculate_z(x, y):
    """
    Calculates an optimal z value of a given 2D point for NICO to touch the screen using a calibration matrix.

    Args:
        x (float): x of a point in sim
        y (float): y of a point in sim

    Returns:
        float: z of a point in sim
    """

    i1, i2 = get_boundary_indexes(x, 0)
    j1, j2 = get_boundary_indexes(y, 1)

    x1_z = get_middle_z(calibration_matrix[i1][j1][0],
                        calibration_matrix[i2][j1][0],
                        x,
                        calibration_matrix[i1][j1][2],
                        calibration_matrix[i2][j1][2])

    x2_z = get_middle_z(calibration_matrix[i1][j2][0],
                        calibration_matrix[i2][j2][0],
                        x,
                        calibration_matrix[i1][j2][2],
                        calibration_matrix[i2][j2][2])

    z = get_middle_z(calibration_matrix[i1][j1][1],
                     calibration_matrix[i1][j2][1],
                     y,
                     x1_z,
                     x2_z)

    return z
