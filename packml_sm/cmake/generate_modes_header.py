#!/usr/bin/env python3
# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ---
"""
Generate a C++ header file (and optionally a Python module) with PackML mode
constants from a YAML definition file.

Usage:
  generate_modes_header.py <yaml_file> <output_header> [<output_python>]

The YAML file should have the following format:
  modes:
    Invalid: 0
    Production: 1
    Maintenance: 2

The generated header will define constants in the packml_modes namespace:
  namespace packml_modes {
    constexpr packml_sm::ModeType Invalid = 0;
    constexpr packml_sm::ModeType Production = 1;
    constexpr packml_sm::ModeType Maintenance = 2;
  }  // namespace packml_modes

When <output_python> is given, a Python module is also generated:
  INVALID = 0
  PRODUCTION = 1
  MAINTENANCE = 2
  ALL_MODES: dict[str, int] = {"Invalid": 0, "Production": 1, ...}
"""

import os
import sys


def generate_modes_header(yaml_file, output_file):
    modes = _parse_modes_yaml(yaml_file)

    # output_file is expected to already contain the packml_sm/ subdirectory,
    # e.g. .../packml_modes/include/packml_sm/default_modes.hpp
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

    import hashlib
    path_hash = hashlib.md5(os.path.abspath(yaml_file).encode()).hexdigest()[:8].upper()
    header_guard = (
        os.path.basename(output_file).upper().replace('.', '_').replace('-', '_')
        + '_' + path_hash + '_'
    )

    with open(output_file, 'w') as f:
        f.write(f'// Auto-generated from {os.path.basename(yaml_file)} -- do not edit manually\n')
        f.write(f'#ifndef {header_guard}_\n')
        f.write(f'#define {header_guard}_\n\n')
        f.write('#include <string>\n')
        f.write('#include "packml_sm/common.hpp"\n')
        f.write('#include "packml_sm/modes_registry.hpp"\n\n')
        f.write('namespace packml_modes {\n\n')
        for name, value in modes.items():
            f.write(f'constexpr packml_sm::ModeType {name} = {value};\n')
        f.write('\n')
        # The names and values are handed to packml_sm rather than compiled into a
        # packml_sm::is_known_mode() / to_string() of this header's own. Those are shared
        # symbols: a second generated header defining them is a redefinition, and any public
        # header that included one of them would impose its mode values on every consumer.
        # Registering instead lets several vocabularies coexist and lets packml_ros validate a
        # mode it could not know at compile time. The suffix keeps the variable itself unique.
        f.write('namespace detail {\n')
        f.write('/// Publishes the modes above to packml_sm::is_known_mode() and\n')
        f.write('/// packml_sm::to_string(), for as long as this header is linked in.\n')
        f.write(f'inline const bool kModesRegistered_{path_hash} = packml_sm::register_modes({{\n')
        for name, value in modes.items():
            f.write(f'  {{"{name}", {value}}},\n')
        f.write('});\n')
        f.write('}  // namespace detail\n')
        f.write('\n}  // namespace packml_modes\n\n')
        f.write(f'#endif  // {header_guard}_\n')

    return modes


def generate_modes_python(yaml_file, output_dir, modes=None):
    """Generate a Python package with PackML mode constants from a YAML file.

    Creates output_dir/packml_modes/__init__.py with the constants directly.
    If *modes* is provided (dict name→value) the YAML file is not re-read.
    """
    if modes is None:
        modes = _parse_modes_yaml(yaml_file)

    pkg_dir = os.path.join(os.path.abspath(output_dir), 'packml_modes')
    os.makedirs(pkg_dir, exist_ok=True)

    with open(os.path.join(pkg_dir, '__init__.py'), 'w') as f:
        f.write(f'# Auto-generated from {os.path.basename(yaml_file)} -- do not edit manually\n')
        f.write('"""PackML mode constants (generated from YAML)."""\n\n')
        for name, value in modes.items():
            f.write(f'{name.upper()}: int = {value}\n')
        f.write('\n')
        items = ', '.join(f'"{name}": {value}' for name, value in modes.items())
        f.write(f'ALL_MODES: dict[str, int] = {{{items}}}\n')
        f.write('\n\n')
        f.write('def mode_to_string(mode: int) -> str:\n')
        f.write('    """Convert a mode value to its string name."""\n')
        f.write('    for name, val in ALL_MODES.items():\n')
        f.write('        if val == mode:\n')
        f.write('            return name\n')
        f.write('    return str(mode)\n')


def _parse_modes_yaml(yaml_file):
    """Parse modes from a YAML file and return as an ordered dict."""
    try:
        import yaml
    except ImportError:
        yaml = None

    with open(yaml_file, 'r') as f:
        content = f.read()

    if yaml is not None:
        data = yaml.safe_load(content)
    else:
        data = {'modes': {}}
        in_modes = False
        for line in content.splitlines():
            stripped = line.strip()
            if stripped == 'modes:':
                in_modes = True
                continue
            if in_modes and stripped and not stripped.startswith('#'):
                if ':' in stripped:
                    name, value = stripped.split(':', 1)
                    try:
                        data['modes'][name.strip()] = int(value.strip())
                    except ValueError:
                        pass

    modes = data.get('modes', {})
    if not modes:
        raise ValueError(f"No 'modes' key found in YAML file: {yaml_file}")
    return modes


if __name__ == '__main__':
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print(f'Usage: {sys.argv[0]} <yaml_file> <output_header> [<python_output_dir>]',
              file=sys.stderr)
        sys.exit(1)
    generate_modes_header(sys.argv[1], sys.argv[2])
    if len(sys.argv) == 4:
        generate_modes_python(sys.argv[1], sys.argv[3])
