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

"""setup.py support for ament_python packages that own an error_map.yaml.

An ament_cmake bringup package aggregates its error catalog by calling
packml_ros_aggregate_error_catalog() from CMake. An ament_python package has
no CMakeLists.txt to hang that off, so it calls aggregate_error_catalog() from
its setup.py instead::

    from packml_ros_setup import aggregate_error_catalog

    generated = aggregate_error_catalog(__file__)

    setup(
        ...
        data_files=[
            ('share/' + package_name + '/config',
                glob.glob('config/*.yaml') + generated),
        ],
    )

Both paths run the same tool (aggregate_error_catalog.py) with the same lint
and the same build-failing exit status, so an ament_python package cannot
silently ship a broken catalog either.
"""

import os
import shutil
import subprocess
import sys

DEFAULT_ERROR_MAP = os.path.join('config', 'error_map.yaml')
DEFAULT_OUTPUT_DIR = os.path.join('config', 'generated')
PRIMARY_OUTPUT = 'machine_error_catalog.yaml'


def find_aggregation_script():
    """Absolute path to aggregate_error_catalog.py, or None if unavailable.

    Same two-step resolution as the CMake function: the installed copy under
    share/packml_ros/cmake/ first, then this module's own source-tree sibling
    (which is how packml_ros's tests reach it before install).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(
            get_package_share_directory('packml_ros'), 'cmake',
            'aggregate_error_catalog.py')
    except Exception:
        installed = None
    if installed and os.path.isfile(installed):
        return installed
    sibling = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'aggregate_error_catalog.py')
    return sibling if os.path.isfile(sibling) else None


def aggregate_error_catalog(setup_file, error_map=DEFAULT_ERROR_MAP,
                            output_dir=DEFAULT_OUTPUT_DIR, formats=None,
                            strict=False, check_instances=None):
    """(Re)generate a package's error catalog from its error_map.yaml.

    Emits machine_error_catalog.yaml (the manager's runtime artifact) plus the
    machine_errors.{json,md} sheets, and returns their paths relative to the
    package directory, ready to append to a data_files entry.

    :param setup_file: the calling setup.py's ``__file__``. Symlinks are
        resolved: ``colcon build --symlink-install`` runs ``setup.py develop``
        against a build-directory *symlink* of setup.py, where only data_files
        are staged, so a path taken literally would not find error_map.yaml.
    :param error_map: integration map, relative to the package directory.
    :param output_dir: where artifacts are written, relative to the package
        directory. This is inside the *source* tree on purpose: ``setup.py
        develop`` never runs a build step, so output written only to a build
        directory would silently never exist for that (common) workflow.
    :param formats: output formats (default: the tool's own yaml, json, md).
    :param strict: promote every lint warning to a build-failing error.
    :param check_instances: instance ids that must have a configured label.

    A misconfigured map (a numbering collision, a broken catalog reference, or
    any other lint error) exits non-zero, which raises out of setup.py and
    aborts the build -- the same guarantee the CMake path gives.
    """
    package_dir = os.path.dirname(os.path.realpath(setup_file))
    out_dir = os.path.join(package_dir, output_dir)
    script = find_aggregation_script()

    if script is None:
        # packml_ros is not on AMENT_PREFIX_PATH (its workspace is not sourced
        # in this shell). Reusing an earlier run's output is fine -- it only
        # goes stale if error_map.yaml or the referenced catalogs changed since
        # -- but with no output at all the install would ship without a
        # catalog, so that is a hard error rather than a silent skip.
        if os.path.isfile(os.path.join(out_dir, PRIMARY_OUTPUT)):
            print('warning: aggregate_error_catalog.py not found -- reusing '
                  'previously generated error catalog (source the packml_ros2 '
                  'workspace to regenerate)', file=sys.stderr)
            return _generated_files(package_dir, out_dir)
        raise RuntimeError(
            'aggregate_error_catalog.py not found and no previously generated '
            'error catalog exists -- source the packml_ros2 workspace and '
            'rebuild')

    # Regenerate from scratch: the caller globs everything in out_dir, so a
    # file left behind by an earlier run (e.g. an output format that no longer
    # exists) must not keep getting installed forever.
    shutil.rmtree(out_dir, ignore_errors=True)
    command = [sys.executable, script,
               '--map', os.path.join(package_dir, error_map),
               '--output-dir', out_dir]
    if formats:
        command += ['--formats', ','.join(formats)]
    if strict:
        command += ['--strict']
    if check_instances:
        command += ['--check-instances', ','.join(check_instances)]
    subprocess.check_call(command)
    return _generated_files(package_dir, out_dir)


def _generated_files(package_dir, out_dir):
    """Everything in out_dir, relative to package_dir, in a stable order."""
    return sorted(
        os.path.relpath(os.path.join(out_dir, name), package_dir)
        for name in os.listdir(out_dir))
