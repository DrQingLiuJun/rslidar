"""Site-specific static obstacle masks retained from the deployed project."""


def filter_static_obstacles(points):
    mask1 = (
        (points[:, 0] > 16.5)
        & (points[:, 0] < 26.5)
        & (points[:, 1] > -2.8)
        & (points[:, 1] < 7.2)
        & (points[:, 2] > -5.0)
        & (points[:, 2] < -3.0)
    )
    mask2 = (
        (points[:, 0] > 14.5)
        & (points[:, 0] < 15.5)
        & (points[:, 1] > 4.0)
        & (points[:, 1] < 5.5)
    )
    mask3 = (
        (points[:, 0] > 20.0)
        & (points[:, 0] < 25.0)
        & (points[:, 1] > -24.0)
        & (points[:, 1] < -21.0)
        & (points[:, 2] > -6.0)
        & (points[:, 2] < -4.5)
    )
    return points[~(mask1 | mask2 | mask3)]
