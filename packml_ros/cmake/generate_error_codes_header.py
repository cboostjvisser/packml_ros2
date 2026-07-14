#!/usr/bin/env python3
# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ---
"""
Generate a C++ header (and optionally a Python module) with strictly-typed error
code constants from an error catalog YAML file.

This mirrors packml_sm/cmake/generate_modes_header.py but reads the `error_codes:`
section of an error catalog file.  Only the integer codes are generated here — the
descriptions are loaded at runtime so translations can change without recompiling.

Usage:
  generate_error_codes_header.py <yaml_file> <output_header> <namespace> [<python_output_dir>]

The YAML file should have at least:
  error_codes:
    OVERCURRENT: 301
    OVER_TEMPERATURE: 302
  descriptions:           # optional here; used only for a build-time warning
    OVERCURRENT:
      ...

The generated header defines constants in the given namespace:
  namespace <namespace> {
    constexpr int32_t OVERCURRENT = 301;
    constexpr int32_t OVER_TEMPERATURE = 302;
  }  // namespace <namespace>
"""

import hashlib
import os
import sys

ERROR_CODES_KEY = 'error_codes'
DESCRIPTIONS_KEY = 'descriptions'

# Every other error-catalog YAML section/field name. Generated into a C++
# header (see generate_schema_keys_header) so the C++ loader (error_catalog.cpp)
# reads its key constants from the same source as this script, instead of a
# second hand-maintained copy.
NODE_NAME_KEY = 'node_name'
LANGUAGES_KEY = 'languages'
INSTANCES_KEY = 'instances'
RESERVED_KEY = 'reserved'
GLOBAL_KEY = 'global'
NODES_KEY = 'nodes'
CODES_KEY = 'codes'
NAME_KEY = 'name'
ACTION_KEY = 'action'
SEVERITY_KEY = 'severity'
CATEGORY_KEY = 'category'
INSTANCED_KEY = 'instanced'

# C++ constant name -> value, in generation order, for generate_schema_keys_header().
_SCHEMA_KEY_CONSTANTS = {
    'kNodeNameKey': NODE_NAME_KEY,
    'kErrorCodesKey': ERROR_CODES_KEY,
    'kDescriptionsKey': DESCRIPTIONS_KEY,
    'kLanguagesKey': LANGUAGES_KEY,
    'kInstancesKey': INSTANCES_KEY,
    'kReservedKey': RESERVED_KEY,
    'kGlobalKey': GLOBAL_KEY,
    'kNodesKey': NODES_KEY,
    'kCodesKey': CODES_KEY,
    'kNameKey': NAME_KEY,
    'kActionKey': ACTION_KEY,
    'kSeverityKey': SEVERITY_KEY,
    'kCategoryKey': CATEGORY_KEY,
    'kInstancedKey': INSTANCED_KEY,
}


def _parse_catalog_yaml(yaml_file):
    """Return (error_codes dict, described_names set) from the YAML file."""
    try:
        import yaml
    except ImportError:
        yaml = None

    with open(yaml_file, 'r') as f:
        content = f.read()

    if yaml is not None:
        data = yaml.safe_load(content) or {}
        codes = data.get(ERROR_CODES_KEY, {}) or {}
        described = set((data.get(DESCRIPTIONS_KEY, {}) or {}).keys())
    else:
        # Minimal fallback parser (no PyYAML): only the error_codes block.
        codes = {}
        described = set()
        section = None
        for line in content.splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            indent = len(line) - len(line.lstrip())
            stripped = line.strip()
            if indent == 0:
                section = stripped[:-1] if stripped.endswith(':') else None
                continue
            if section == ERROR_CODES_KEY and ':' in stripped:
                name, value = stripped.split(':', 1)
                try:
                    codes[name.strip()] = int(value.strip())
                except ValueError:
                    pass
            elif section == DESCRIPTIONS_KEY and indent == 2 and stripped.endswith(':'):
                described.add(stripped[:-1].strip())

    codes = {str(name): int(value) for name, value in codes.items()}
    if not codes:
        raise ValueError(f"No '{ERROR_CODES_KEY}' section found in YAML file: {yaml_file}")
    return codes, described


def _warn_missing_descriptions(codes, described, yaml_file):
    """Emit a build warning for any error code lacking a description block."""
    missing = [name for name in codes if name not in described]
    if missing and described:  # only warn when a descriptions section exists at all
        for name in missing:
            sys.stderr.write(
                f"warning: error code '{name}' in {os.path.basename(yaml_file)} "
                f"has no description block\n")


def generate_error_codes_header(yaml_file, output_file, namespace):
    codes, described = _parse_catalog_yaml(yaml_file)
    _warn_missing_descriptions(codes, described, yaml_file)

    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

    path_hash = hashlib.md5(os.path.abspath(yaml_file).encode()).hexdigest()[:8].upper()
    header_guard = (
        os.path.basename(output_file).upper().replace('.', '_').replace('-', '_')
        + '_' + path_hash + '_'
    )

    with open(output_file, 'w') as f:
        f.write(f'// Auto-generated from {os.path.basename(yaml_file)} -- do not edit manually\n')
        f.write(f'#ifndef {header_guard}_\n')
        f.write(f'#define {header_guard}_\n\n')
        f.write('#include <cstdint>\n\n')
        f.write(f'namespace {namespace} {{\n\n')
        for name, value in codes.items():
            f.write(f'constexpr int32_t {name} = {value};\n')
        f.write(f'\n}}  // namespace {namespace}\n\n')
        f.write(f'#endif  // {header_guard}_\n')

    return codes


def generate_error_codes_python(yaml_file, output_dir, module_name, codes=None):
    """Generate a Python module with error code constants from a YAML file."""
    if codes is None:
        codes, _ = _parse_catalog_yaml(yaml_file)

    pkg_dir = os.path.join(os.path.abspath(output_dir), module_name)
    os.makedirs(pkg_dir, exist_ok=True)

    with open(os.path.join(pkg_dir, '__init__.py'), 'w') as f:
        f.write(f'# Auto-generated from {os.path.basename(yaml_file)} -- do not edit manually\n')
        f.write('"""PackML error code constants (generated from YAML)."""\n\n')
        for name, value in codes.items():
            f.write(f'{name}: int = {value}\n')
        f.write('\n')
        items = ', '.join(f'"{name}": {value}' for name, value in codes.items())
        f.write(f'ALL_CODES: dict[str, int] = {{{items}}}\n')


def generate_schema_keys_header(output_file):
    """Generate the C++ header of error-catalog YAML schema key constants.

    This is the single source for the section/field names both this script
    and the C++ loader (error_catalog.cpp) read, so the two YAML readers
    can't independently drift on the same format.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

    with open(output_file, 'w') as f:
        f.write('// Auto-generated from generate_error_codes_header.py -- do not edit manually\n')
        f.write('#pragma once\n\n')
        f.write('namespace packml_ros\n{\nnamespace schema\n{\n\n')
        for cpp_name, value in _SCHEMA_KEY_CONSTANTS.items():
            f.write(f'constexpr const char * {cpp_name} = "{value}";\n')
        f.write('\n}  // namespace schema\n}  // namespace packml_ros\n')


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--schema-keys-header':
        generate_schema_keys_header(sys.argv[2])
        sys.exit(0)

    if len(sys.argv) < 4 or len(sys.argv) > 5:
        print(
            f'Usage: {sys.argv[0]} <yaml_file> <output_header> <namespace> '
            f'[<python_output_dir>]\n'
            f'   or: {sys.argv[0]} --schema-keys-header <output_file>',
            file=sys.stderr)
        sys.exit(1)

    yaml_path = sys.argv[1]
    header_path = sys.argv[2]
    ns = sys.argv[3]
    generated = generate_error_codes_header(yaml_path, header_path, ns)
    if len(sys.argv) == 5:
        generate_error_codes_python(yaml_path, sys.argv[4], ns.lower(), generated)
