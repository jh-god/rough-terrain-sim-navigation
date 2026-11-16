"""Select perception use without changing the shared Nav2 parameter file."""
from copy import deepcopy


def nav2_for_mode(parameters, obstacle_detection):
    result = deepcopy(parameters)
    if not obstacle_detection:
        for name in ['global_costmap', 'local_costmap']:
            costmap = result[name][name]['ros__parameters']
            costmap['plugins'] = [p for p in costmap['plugins'] if p != 'obstacle_layer']
            costmap.pop('obstacle_layer', None)
    return result
