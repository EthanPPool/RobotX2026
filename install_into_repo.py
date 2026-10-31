#!/usr/bin/env python3
"""Add this interpretation package to an existing, possibly offline-edited repo.

Preflight all conflicts before writing. Existing packages are not replaced;
only the coordination interface message list is extended in place.
"""
import argparse
from pathlib import Path
import shutil


MESSAGES = ('BuoyColorDetection', 'BuoyColorDetectionArray', 'BuoyObservation',
            'BuoyObservationArray', 'BuoyTrack', 'BuoyTrackArray')


def planned_changes(source, target):
    source, target = Path(source).resolve(), Path(target).resolve()
    if source == target:
        raise ValueError('Choose the actual device repository, not this extracted bundle')
    interface = Path('coordination_ws/src/robotx_coordination_interfaces')
    cmake = target / interface / 'CMakeLists.txt'
    if not cmake.is_file() or not (target / 'boat_ws/src/boat_interfaces').is_dir():
        raise ValueError('Target must contain the existing coordination and boat interface packages')
    paths = []
    for folder in ('coordination_ws/src/robotx_buoy_interpretation', 'tests/buoy_interpretation'):
        paths.extend(p.relative_to(source) for p in (source/folder).rglob('*')
                     if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc')
    paths.extend(interface/'msg'/f'{name}.msg' for name in MESSAGES)
    paths.extend(Path(p) for p in ('install_into_repo.py', 'scripts/build_buoy_interpretation.sh',
                 'scripts/test_buoy_interpretation.sh', 'docs/BUOY_INTERPRETATION.md'))
    copies = []
    for relative in sorted(paths):
        origin, destination = source/relative, target/relative
        if not origin.is_file():
            raise ValueError(f'Incomplete bundle: {relative}')
        if destination.exists():
            if not destination.is_file() or origin.read_bytes() != destination.read_bytes():
                raise ValueError(f'Existing file differs; resolve before installing: {relative}')
        else:
            copies.append(relative)
    old = cmake.read_text()
    needle = 'rosidl_generate_interfaces(${PROJECT_NAME}'
    if old.count(needle) != 1:
        raise ValueError('Could not identify a unique coordination interface declaration')
    missing = [f'  "msg/{name}.msg"' for name in MESSAGES
               if f'"msg/{name}.msg"' not in old]
    new = old.replace(needle, needle+'\n'+'\n'.join(missing), 1) if missing else old
    return copies, cmake, old, new


def install(source, target, apply=False):
    copies, cmake, old, new = planned_changes(source, target)
    if apply:
        for relative in copies:
            destination = Path(target)/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(Path(source)/relative, destination)
        if new != old:
            cmake.write_text(new)
    return len(copies), new != old


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--apply', action='store_true', help='Write after successful preflight')
    args = parser.parse_args()
    try:
        count, cmake_changed = install(Path(__file__).resolve().parent, args.repo, args.apply)
    except (ValueError, OSError) as exc:
        parser.exit(2, str(exc)+'\n')
    verb = 'Installed' if args.apply else 'Preflight passed for'
    print(f'{verb} {count} new files; coordination CMake extension: {cmake_changed}.')
    if not args.apply:
        print('Add --apply to install. No files changed.')


if __name__ == '__main__':
    main()
