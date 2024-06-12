def tab2sim(y, x):
    """
    Converts given touchscreen coordinates into simulation coordinates

    Args:
        y (float): y in pixels
        x (float): x in pixels

    Returns:
        tuple (float, float): x and y of a given point in sim
    """

    if y > 1920 or y < 0 or x > 1080 or x < 0:
        print("Invalid tablet coordinates")
        quit()

    y_ratio = 1 - y / 1920
    x_ratio = 1 - x / 1080
    y_sim = -0.26 + y_ratio * 0.52
    x_sim = 0.25 + x_ratio * 0.32

    return x_sim, y_sim


def sim2tab(x, y):
    """
    Converts given simulation coordinates into touchscreen coordinates

    Args:
        y (float): y in sim
        x (float): x in sim

    Returns:
        tuple (float, float): x and y of a given point in pixels
    """

    if x > 0.57 or x < 0.25 or y > 0.26 or y < -0.26:
        print("Invalid sim coordinates for touchscreen")
        quit()

    x_ratio = 1 - (x - 0.25) / 0.32
    y_ratio = 1 - (y - -0.26) / 0.52
    x_tab = x_ratio * 1080
    y_tab = y_ratio * 1920

    return y_tab, x_tab


def pix2cm(pixels):
    """
    Converts given pixels into centimeters

    Args:
        pixels (float): number of pixels to convert to centimeters

    Returns:
        float: centimeters
    """

    ratio = 47.5 / 1920

    return pixels * ratio
