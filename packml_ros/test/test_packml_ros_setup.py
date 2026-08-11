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

"""Tests for the setup.py-side error-catalog helper.

Pure Python, no ROS node required. Each test builds a throwaway ament_python
package (a setup.py stub plus config/error_map.yaml) in tmp_path and calls the
helper the way a real setup.py would.
"""

import importlib.util
import os
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODULE = os.path.join(_HERE, '..', 'scripts', 'packml_ros_setup.py')

_spec = importlib.util.spec_from_file_location('packml_ros_setup', _MODULE)
helper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(helper)


def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)
    return path


def make_package(root, node_code=1):
    """Write a minimal ament_python package; return its setup.py path."""
    write(os.path.join(root, 'config', 'motor.yaml'), """
node_name: motor
error_codes:
  OVERTEMP: 1
descriptions:
  OVERTEMP: { action: HOLD, severity: ERROR, en: "motor over temperature" }
""")
    write(os.path.join(root, 'config', 'error_map.yaml'), f"""
languages: [en]
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "heartbeat lost" }}
nodes:
  motor:
    catalog: {os.path.join(root, 'config', 'motor.yaml')}
    base: {node_code * 1000}
""")
    return write(os.path.join(root, 'setup.py'), '# stub\n')


def test_generates_default_formats_into_the_source_tree(tmp_path):
    setup_file = make_package(str(tmp_path))

    generated = helper.aggregate_error_catalog(setup_file)

    assert generated == [
        os.path.join('config', 'generated', 'machine_error_catalog.yaml'),
        os.path.join('config', 'generated', 'machine_errors.json'),
        os.path.join('config', 'generated', 'machine_errors.md'),
    ]
    for relative in generated:
        assert os.path.isfile(os.path.join(str(tmp_path), relative))


def test_returned_paths_are_relative_so_data_files_can_use_them(tmp_path):
    setup_file = make_package(str(tmp_path))

    generated = helper.aggregate_error_catalog(setup_file)

    assert not any(os.path.isabs(path) for path in generated)


def test_resolves_a_symlinked_setup_py_to_the_real_package(tmp_path):
    """`colcon build --symlink-install` runs a build-dir symlink of setup.py."""
    source = str(tmp_path / 'src')
    setup_file = make_package(source)
    build_dir = tmp_path / 'build'
    build_dir.mkdir()
    symlinked = str(build_dir / 'setup.py')
    os.symlink(setup_file, symlinked)

    generated = helper.aggregate_error_catalog(symlinked)

    assert os.path.isfile(os.path.join(source, generated[0]))
    assert not os.path.exists(os.path.join(str(build_dir), 'config'))


def test_stale_output_from_an_earlier_run_is_removed(tmp_path):
    setup_file = make_package(str(tmp_path))
    stale = write(
        os.path.join(str(tmp_path), 'config', 'generated', 'machine_errors.csv'),
        'a format that no longer exists\n')

    generated = helper.aggregate_error_catalog(setup_file)

    assert not os.path.exists(stale)
    assert not any(path.endswith('.csv') for path in generated)


def test_selected_formats_are_honoured(tmp_path):
    setup_file = make_package(str(tmp_path))

    generated = helper.aggregate_error_catalog(setup_file, formats=['yaml'])

    assert generated == [
        os.path.join('config', 'generated', 'machine_error_catalog.yaml')]


def test_lint_error_in_the_map_aborts_the_build(tmp_path):
    setup_file = make_package(str(tmp_path))
    # Two nodes sharing one base window: colliding global codes (a lint error,
    # not a warning) must fail the caller's setup.py.
    write(os.path.join(str(tmp_path), 'config', 'error_map.yaml'), f"""
languages: [en]
nodes:
  motor:
    catalog: {os.path.join(str(tmp_path), 'config', 'motor.yaml')}
    base: 1000
  motor_b:
    catalog: {os.path.join(str(tmp_path), 'config', 'motor.yaml')}
    base: 1000
""")

    with pytest.raises(subprocess.CalledProcessError):
        helper.aggregate_error_catalog(setup_file)


def test_strict_promotes_warnings_to_a_failed_build(tmp_path):
    setup_file = make_package(str(tmp_path))
    write(os.path.join(str(tmp_path), 'config', 'error_map.yaml'),
          'languages: [en]\n')  # no reserved heartbeat_timeout: a warning

    helper.aggregate_error_catalog(setup_file)  # tolerated without strict

    with pytest.raises(subprocess.CalledProcessError):
        helper.aggregate_error_catalog(setup_file, strict=True)


def test_missing_tool_reuses_previous_output_with_a_warning(
        tmp_path, capsys, monkeypatch):
    setup_file = make_package(str(tmp_path))
    generated = helper.aggregate_error_catalog(setup_file)
    monkeypatch.setattr(helper, 'find_aggregation_script', lambda: None)

    reused = helper.aggregate_error_catalog(setup_file)

    assert reused == generated
    assert 'reusing' in capsys.readouterr().err


def test_missing_tool_and_no_previous_output_is_a_hard_error(
        tmp_path, monkeypatch):
    setup_file = make_package(str(tmp_path))
    monkeypatch.setattr(helper, 'find_aggregation_script', lambda: None)

    with pytest.raises(RuntimeError, match='source the packml_ros2 workspace'):
        helper.aggregate_error_catalog(setup_file)


def test_finds_the_aggregation_script(tmp_path):
    assert os.path.isfile(helper.find_aggregation_script())
