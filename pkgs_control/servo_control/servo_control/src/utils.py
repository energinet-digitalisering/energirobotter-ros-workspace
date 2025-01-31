def interval_map(x, x0, x1, y0, y1):

    # x: value
    # [x0, x1]: original interval
    # [y0, y1]: target interval

    if x0 == x1:
        return y0  # Avoid division by zero; return y0 by default

    # Clamp x within [x0, x1] to avoid out-of-bounds mapping
    x = max(min(x, x1), x0)

    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0)
