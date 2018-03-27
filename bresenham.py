import numpy as np


def bresenham(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    if x2 - x1 > 0 and y2 - y1 > 0:
        return _bresenham_upper_left(p1, p2)
    elif x2 - x1 < 0 and y2 - y1 < 0:
        return _bresenham_upper_left(p2, p1)
    elif x2 - x1 > 0 and y2 - y1 < 0:
        return _bresenham_lower_left(p1, p2)
    elif x2 - x1 <0 and y2 - y1 > 0:
        return _bresenham_lower_left(p2, p1)
    elif x2 - x1 > 0 and y2 - y1 == 0:
        return _bresenham_horizontal(p1, p2)
    elif x2 - x1 < 0 and y2 - y1 == 0:
        return _bresenham_horizontal(p2, p1)
    elif x2 - x1 == 0 and y2 - y1 > 0:
        return _bresenham_vertical(p1, p2)
    elif x2 - x1 == 0 and y2 - y1 < 0:
        return _bresenham_vertical(p2, p1)
    else:
        return np.array([])


def _bresenham_upper_left(p1, p2):
    """
    Case where x1 < x2 and y1 < y2
    """
    cells = []

    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1

    d = 0
    x = x1
    y = y1
    while x < x2 and y < y2:
        cells.append([x, y])
        if d < dx - dy:
            x += 1
            d += dy
        else:
            y += 1
            d -= dx

    return np.array(cells)


def _bresenham_lower_left(p1, p2):
    """
    Case where x1 < x2 and y1 > y2
    """
    cells = []

    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1

    d = dx
    x = x1
    y = y1-1
    while x < x2 and y > y2-1:
        cells.append([x, y])
        if d > -dy:
            x += 1
            d += dy
        else:
            y -= 1
            d += dx

    return np.array(cells)


def _bresenham_horizontal(p1, p2):
    cells = []
    x1, y1 = p1
    x2, y2 = p2
    for x in range(x1, x2):
        cells.append([x, y1 - 1])
        cells.append([x, y1])

    return np.array(cells)


def _bresenham_vertical(p1, p2):
    cells = []
    x1, y1 = p1
    x2, y2 = p2
    for y in range(y1, y2):
        cells.append([x1, y])
        cells.append([x1-1, y])

    return np.array(cells)
