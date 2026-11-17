"""Sensor publication is independent of obstacle-perception mode."""
from pathlib import Path
import xml.etree.ElementTree as ET


def camera_enabled(option, attached):
    option = option.lower()
    if option not in ('auto', 'true', 'false'):
        raise ValueError('camera must be auto, true or false')
    if option == 'true' and not attached:
        raise ValueError('selected upstream configuration has no RGB-D camera; use camera:=auto or false')
    return attached if option == 'auto' else option == 'true'


def sensor_world(source, output, navsat):
    """Add GPS infrastructure to a private copy, preserving the simulator source.

    GPS uses a synthetic WGS84 origin (0 degrees, 0 degrees, 0 metres), ENU.
    This is a simulation reference, not the geographic position of the terrain.
    """
    if not navsat:
        return str(source)
    root = ET.parse(source)
    world = root.getroot().find('world')
    if not any(p.get('name', '').endswith('::NavSat') for p in world.findall('plugin')):
        ET.SubElement(world, 'plugin', filename='libignition-gazebo-navsat-system.so',
                      name='ignition::gazebo::systems::NavSat')
    if world.find('spherical_coordinates') is None:
        coordinates = ET.SubElement(world, 'spherical_coordinates')
        for key, value in dict(surface_model='EARTH_WGS84', world_frame_orientation='ENU',
                               latitude_deg='0', longitude_deg='0', elevation='0', heading_deg='0').items():
            ET.SubElement(coordinates, key).text = value
    result = Path(output)/'sensor_world.sdf'
    root.write(result, encoding='unicode', xml_declaration=True)
    return str(result)
