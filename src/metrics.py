import math


def calculate_position_error(actual_x, actual_y, target_x, target_y):
    """
    Calculate the Euclidean distance between the actual end-effector
    position and the target position.

    Args:
        actual_x: Actual end-effector x-coordinate.
        actual_y: Actual end-effector y-coordinate.
        target_x: Target x-coordinate.
        target_y: Target y-coordinate.

    Returns:
        The Cartesian position error.
    """
    return math.hypot(
        actual_x - target_x,
        actual_y - target_y,
    )


def calculate_path_length(points):
    """
    Calculate the total Cartesian distance travelled along a path.

    Args:
        points: Sequence of (x, y) coordinate tuples.

    Returns:
        The total path length.
    """
    total_length = 0.0

    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]

        total_length += math.hypot(
            x2 - x1,
            y2 - y1,
        )

    return total_length

def calculate_max_step_distance(points):
    """
    Calculate the maximum Cartesian distance travelled between
    consecutive points along a path.

    Args:
        points: Sequence of (x, y) coordinate tuples.

    Returns:
        The maximum distance between consecutive path points.
    """
    if len(points) < 2:
        return 0.0

    max_distance = 0.0

    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]

        step_distance = math.hypot(
            x2 - x1,
            y2 - y1,
        )

        max_distance = max(max_distance, step_distance)

    return max_distance

def calculate_rms_error(actual_values, target_value):
    """
    Calculate RMS error between actual values and a constant target.
    """
    if len(actual_values) == 0:
        return 0.0

    squared_errors = [
        (actual - target_value) ** 2
        for actual in actual_values
    ]

    mean_squared_error = sum(squared_errors) / len(squared_errors)

    return math.sqrt(mean_squared_error)