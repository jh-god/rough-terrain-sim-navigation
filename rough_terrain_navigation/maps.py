"""Generate an empty, bounded metric map; never derive risk from elevation."""
import argparse
import math
from pathlib import Path
import yaml


def generate_map(directory, width=50., height=50., resolution=0.1,
                 origin_x=-25., origin_y=-25., boundary=0.3):
    values = [width, height, resolution, origin_x, origin_y, boundary]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Map parameters must be finite')
    if min(width, height, resolution) <= 0 or boundary < 0:
        raise ValueError('Map size/resolution must be positive and boundary nonnegative')
    nx, ny = round(width/resolution), round(height/resolution)
    if not (math.isclose(nx*resolution, width, abs_tol=1e-7)
            and math.isclose(ny*resolution, height, abs_tol=1e-7)):
        raise ValueError('Map dimensions must be multiples of resolution')
    edge = math.ceil(boundary/resolution)
    if nx*ny > 25_000_000 or min(nx, ny) <= 2*edge:
        raise ValueError('Map too large or boundary consumes map')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # PGM rows run from north to south; uniform boundary is symmetric.
    row = bytes([0])*edge + bytes([254])*(nx-2*edge) + bytes([0])*edge
    data = bytes(nx*edge) + row*(ny-2*edge) + bytes(nx*edge)
    (directory/'empty.pgm').write_bytes(f'P5\n{nx} {ny}\n255\n'.encode()+data)
    metadata = dict(image='empty.pgm', mode='trinary', resolution=resolution,
                    origin=[origin_x, origin_y, 0.0], negate=0,
                    occupied_thresh=0.65, free_thresh=0.25)
    target = directory/'empty.yaml'
    target.write_text(yaml.safe_dump(metadata, sort_keys=False))
    return str(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    for name, value in [('width', 50.), ('height', 50.), ('resolution', .1),
                        ('origin-x', -25.), ('origin-y', -25.), ('boundary', .3)]:
        parser.add_argument('--'+name, type=float, default=value)
    args = vars(parser.parse_args())
    print(generate_map(args.pop('output_dir'), **args))
