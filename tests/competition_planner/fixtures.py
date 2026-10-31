from robotx_competition_planner.models import Marker, PlannerConfig, Pose


BOUNDARY = ((-10., -14.), (36., -14.), (36., 14.), (-10., 14.))


def markers(source='usv', stamp=10.):
    specs = [(0., 0., 'blue', 'flashing'), (26., 0., 'blue', 'steady'),
             (9., 3.5, 'green', 'flashing'), (17., 3.5, 'green', 'flashing'),
             (10., -3.5, 'red', 'flashing'), (18., -3.5, 'red', 'flashing'),
             (13., 0., 'black', 'inactive'), (4., 8., 'black', 'inactive'),
             (21., 8., 'black', 'inactive'), (23., -8., 'black', 'inactive')]
    return tuple(Marker(i+1, x, y, .25, .15, color, state, .95, stamp, source)
                 for i, (x, y, color, state) in enumerate(specs))


def pose(stamp=10.):
    return Pose(-6., 0., 0., stamp)
