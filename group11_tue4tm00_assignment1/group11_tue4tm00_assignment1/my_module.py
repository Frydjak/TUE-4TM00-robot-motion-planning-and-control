
def join_with_spaces(*args):
    """
    Concatenates a variable number of inputs into a single string,
    with each input separated by a space.

    Parameters:
    *args: A variable number of arguments of any type.

    Returns:
    A single string containing all inputs concatenated and separated by spaces.
    """
    return ' '.join(str(arg) for arg in args)
