def interval_map(x, x0, x1, y0, y1):

    # x: value
    # [x0, x1]: original interval
    # [y0, y1]: target interval

    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0)
