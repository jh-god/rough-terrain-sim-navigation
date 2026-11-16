"""Select perception use without changing the shared Nav2 parameter file."""
from copy import deepcopy


def obstacle_detection_for_mode(mode, override='auto'):
    """Resolve the public mode and the legacy explicit detection override."""
    # Keep the first branch version's mode name as an input alias only.
    if mode == 'waypoint':
        mode = 'map_only'
    if mode not in ['perception', 'map_only']:
        raise ValueError('mode must be perception or map_only')
    if override not in ['auto', 'true', 'false']:
        raise ValueError('obstacle_detection must be auto, true or false')
    return mode == 'perception' if override == 'auto' else override == 'true'


def nav2_for_mode(parameters, obstacle_detection):
    result = deepcopy(parameters)
    if not obstacle_detection:
        for name in ['global_costmap', 'local_costmap']:
            costmap = result[name][name]['ros__parameters']
            costmap['plugins'] = [p for p in costmap['plugins'] if p != 'obstacle_layer']
            costmap.pop('obstacle_layer', None)
    return result
