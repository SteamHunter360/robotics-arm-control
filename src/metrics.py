import math


def calculate_position_error(actual_x, actual_y, target_x, target_y):
    return math.hypot(actual_x - target_x, actual_y - target_y)