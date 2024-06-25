def rescale_value(x):
    """
    Rescale a value from the range [-180, 180] to the range [-30, 50] where 0 maps to 0.
    
    Parameters:
    x (float): The value in the original range.
    
    Returns:
    float: The value in the new range.
    """
    # Scaling factor
    a = 2 / 9
    
    # Shift constant
    b = -10
    
    # Apply the transformation
    y = a * x + b
    
    return y

# Example usage
original_values = [-180, -90, 0, 90, 180]
rescaled_values = [rescale_value(x) for x in original_values]

print("Original values:", original_values)
print("Rescaled values:", rescaled_values)