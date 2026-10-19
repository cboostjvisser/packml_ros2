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

"""Tests for the offline error-catalog aggregation tool.

Pure Python, no ROS node required.
"""

import importlib.util
import json
import os
import subprocess
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPT = os.path.join(_HERE, '..', 'scripts', 'aggregate_error_catalog.py')

_spec = importlib.util.spec_from_file_location('aggregate_error_catalog', _SCRIPT)
agg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(agg)


def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)
    return path


@pytest.fixture
def workdir(tmp_path):
    return tmp_path


def _codes(issues):
    return [i.code for i in issues]


def _minimal_node_yaml(path, node_name, code=1):
    """Write the smallest valid per-node catalog: one HOLD-severity code.

    Used by tests that only care about a node's window/global assignment,
    not its catalog content.
    """
    return write(path, (
        f'node_name: {node_name}\n'
        f'error_codes:\n  X: {code}\n'
        f'descriptions:\n  X: {{ action: HOLD, en: {node_name} }}\n'))


# ---------------------------------------------------------------------------
# Basic aggregation
# ---------------------------------------------------------------------------

def test_base_plus_local_assigns_global(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: motor_driver_node
error_codes:
  OVERCURRENT: 301
descriptions:
  OVERCURRENT:
    action: ABORT
    severity: CRITICAL
    en: "Motor overcurrent detected"
""")
    write(str(workdir / 'map.yaml'), f"""
languages: [en]
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "Heartbeat lost" }}
nodes:
  motor_driver_node:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, warnings = agg.apply_strict(issues, strict=False)
    assert errors == []
    entry = catalog.nodes['motor_driver_node']['codes'][301]
    assert entry['global'] == 1301
    assert entry['descriptions']['en'] == 'Motor overcurrent detected'


def test_remap_overrides_base_plus_local(workdir):
    write(str(workdir / 'gripper.yaml'), """
node_name: acme_gripper_node
error_codes:
  WILD_FAULT: 30100
descriptions:
  WILD_FAULT:
    action: HOLD
    severity: ERROR
    en: "Gripper fault"
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  acme_gripper_node:
    catalog: {workdir / 'gripper.yaml'}
    base: 5000
    remap: {{ 30100: 5090 }}
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert errors == []
    assert catalog.nodes['acme_gripper_node']['codes'][30100]['global'] == 5090


def test_d8_override_merges_locale_text_keeping_other_locales(workdir):
    write(str(workdir / 'gripper.yaml'), """
node_name: acme_gripper_node
error_codes:
  WILD_FAULT: 30100
descriptions:
  WILD_FAULT:
    action: HOLD
    severity: ERROR
    en: "Gripper vacuum lost"
    nl: "Gripper vacuum verloren"
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  acme_gripper_node:
    catalog: {workdir / 'gripper.yaml'}
    base: 5000
    remap: {{ 30100: 5090 }}
    overrides:
      30100:
        en: "Open guard 3, clear the pouch, press Reset."
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert errors == []
    descriptions = catalog.nodes['acme_gripper_node']['codes'][30100]['descriptions']
    assert descriptions['en'] == 'Open guard 3, clear the pouch, press Reset.'
    assert descriptions['nl'] == 'Gripper vacuum verloren'  # not overridden -> vendor text


def test_reserved_entry_and_instance_label(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en, nl]
reserved:
  heartbeat_timeout:
    global: 1
    action: ABORT
    severity: CRITICAL
    en: "Heartbeat lost from {instance}"
    nl: "Heartbeat verloren van {instance}"
instances:
  cell_north: { en: "North Cell", nl: "Noord Cel" }
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert errors == []
    assert catalog.reserved['heartbeat_timeout']['global'] == 1
    assert catalog.reserved['heartbeat_timeout']['instanced'] is True
    assert catalog.instances['cell_north']['en'] == 'North Cell'


def test_shared_catalog_across_multiple_node_identities_recorded_once(workdir):
    # error_map.yaml's "Node identity" note: two runtime node names may point
    # at the same physical catalog file (e.g. two instances of one role, or —
    # as in the health demo — several different simulated roles sharing one
    # example catalog). Provenance should list that file once, not once per
    # referencing node.
    catalog_path = write(str(workdir / 'shared.yaml'), """
node_name: shared
error_codes:
  X: 1
descriptions:
  X: { action: HOLD, en: "x" }
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {catalog_path}
    base: 1000
  node_b:
    catalog: {catalog_path}
    base: 2000
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert errors == []
    assert catalog.provenance_sources == [(catalog_path, agg._sha256_file(catalog_path))]
    assert catalog.nodes['node_a']['codes'][1]['global'] == 1001
    assert catalog.nodes['node_b']['codes'][1]['global'] == 2001


# ---------------------------------------------------------------------------
# Node-level parsing semantics (mirrors the deleted C++ node loader)
# ---------------------------------------------------------------------------

def test_node_catalog_rejects_nonpositive_code_and_warns(workdir):
    path = write(str(workdir / 'n.yaml'), """
node_name: n
error_codes:
  BAD: 0
  GOOD: 5
descriptions:
  BAD: { action: ABORT, en: "bad" }
  GOOD: { action: HOLD, en: "good" }
""")
    entries, issues = agg.parse_node_catalog(path)
    assert 0 not in entries
    assert 5 in entries
    assert any(i.code == 'reserved-code' for i in issues)


def test_node_catalog_duplicate_local_last_wins(workdir):
    path = write(str(workdir / 'n.yaml'), """
node_name: n
error_codes:
  FIRST: 10
  SECOND: 10
descriptions:
  FIRST: { action: HOLD, en: "first" }
  SECOND: { action: ABORT, en: "second" }
""")
    entries, issues = agg.parse_node_catalog(path)
    assert entries[10]['name'] == 'SECOND'
    assert any(i.code == 'duplicate-local' for i in issues)


def test_node_catalog_orphan_and_missing_description_warn(workdir):
    path = write(str(workdir / 'n.yaml'), """
node_name: n
error_codes:
  REAL: 1
descriptions:
  REAL: { action: HOLD, en: "real" }
  ORPHAN: { action: ABORT, en: "no matching code" }
""")
    entries, issues = agg.parse_node_catalog(path)
    assert 'ORPHAN' not in [e['name'] for e in entries.values()]
    codes = _codes(issues)
    assert 'orphan-description' in codes

    path2 = write(str(workdir / 'n2.yaml'), """
node_name: n2
error_codes:
  DOCUMENTED: 1
  UNDOCUMENTED: 2
descriptions:
  DOCUMENTED: { action: HOLD, en: "has one" }
""")
    _, issues2 = agg.parse_node_catalog(path2)
    assert 'missing-description' in _codes(issues2)


# ---------------------------------------------------------------------------
# Lint: errors (E1-E6)
# ---------------------------------------------------------------------------

def test_window_overlap_between_nodes(workdir):
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    _minimal_node_yaml(str(workdir / 'b.yaml'), 'b')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {workdir / 'a.yaml'}
    base: 1000
    window: 500
  node_b:
    catalog: {workdir / 'b.yaml'}
    base: 1200
    window: 500
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'window-overlap' in _codes(issues)


def test_window_overlaps_reserved_band(workdir):
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {workdir / 'a.yaml'}
    base: 0
    window: 1000
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'window-overlap' in _codes(issues)


def test_local_code_outside_window_without_remap(workdir):
    write(str(workdir / 'a.yaml'), """
node_name: a
error_codes:
  TOO_BIG: 5000
descriptions:
  TOO_BIG: { action: HOLD, en: "oops" }
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {workdir / 'a.yaml'}
    base: 1000
    window: 1000
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'local-code-out-of-range' in _codes(issues)


def test_remap_collides_with_another_nodes_window(workdir):
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    write(str(workdir / 'b.yaml'), """
node_name: b
error_codes:
  WILD: 99999
descriptions:
  WILD: { action: HOLD, en: "wild" }
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {workdir / 'a.yaml'}
    base: 1000
    window: 1000
  node_b:
    catalog: {workdir / 'b.yaml'}
    base: 5000
    window: 1000
    remap: {{ 99999: 1500 }}
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'remap-collision' in _codes(issues)


def test_remap_into_own_window_is_permitted(workdir):
    write(str(workdir / 'b.yaml'), """
node_name: b
error_codes:
  WILD: 99999
descriptions:
  WILD: { action: HOLD, en: "wild" }
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_b:
    catalog: {workdir / 'b.yaml'}
    base: 5000
    window: 1000
    remap: {{ 99999: 5090 }}
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'remap-collision' not in _codes(issues)


def test_duplicate_global_across_nodes(workdir):
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    _minimal_node_yaml(str(workdir / 'b.yaml'), 'b')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  node_a:
    catalog: {workdir / 'a.yaml'}
    base: 1000
    remap: {{ 1: 9000 }}
  node_b:
    catalog: {workdir / 'b.yaml'}
    base: 2000
    remap: {{ 1: 9000 }}
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'duplicate-global' in _codes(issues)


def test_duplicate_global_reserved_vs_reserved(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 1, action: ABORT, en: "a" }
  fault_b: { global: 1, action: ABORT, en: "b" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'duplicate-global' in _codes(issues)


def test_instance_placeholder_mismatch_both_directions(workdir):
    path = write(str(workdir / 'n.yaml'), """
node_name: n
error_codes:
  DOOR: 1
  BADPLAIN: 2
descriptions:
  DOOR: { action: ABORT, instanced: true, en: "Guard door open" }
  BADPLAIN: { action: WARN, en: "Something at {instance}" }
""")
    _, issues = agg.parse_node_catalog(path)
    mismatches = [i for i in issues if i.code == 'instance-placeholder-mismatch']
    assert any('DOOR' in i.message for i in mismatches)
    assert any('BADPLAIN' in i.message for i in mismatches)


def test_global_out_of_int32_range(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  huge: { global: 5000000000, action: ABORT, en: "too big" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'value-out-of-range' in _codes(issues)


# ---------------------------------------------------------------------------
# Lint: warnings and --strict promotion
# ---------------------------------------------------------------------------

def test_description_too_long(workdir):
    write(str(workdir / 'map.yaml'), f"""
reserved:
  fault_a: {{ global: 1, action: ABORT, en: "{'x' * 90}" }}
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'description-too-long' in _codes(issues)


def test_catalog_with_zero_usable_codes(workdir):
    write(str(workdir / 'empty.yaml'), 'node_name: n\nerror_codes: {}\ndescriptions: {}\n')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  n:
    catalog: {workdir / 'empty.yaml'}
    base: 1000
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'catalog-empty' in _codes(issues)


def test_check_instances_missing_label(workdir):
    write(str(workdir / 'map.yaml'), 'languages: [en]\n')
    _, issues = agg.aggregate(str(workdir / 'map.yaml'), check_instances=['cell_north'])
    assert 'instance-missing-label' in _codes(issues)


def test_declared_language_missing_from_entry(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en, nl]
reserved:
  fault_a: { global: 1, action: ABORT, en: "only english" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'description-missing-locale' in _codes(issues)


def test_severity_outside_known_vocabulary(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 1, action: ABORT, severity: SUPER_BAD, en: "x" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'unrecognized-severity' in _codes(issues)


def test_reserved_outside_band(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 500, action: ABORT, en: "x" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'reserved-outside-band' in _codes(issues)


def test_no_heartbeat_timeout_reserved(workdir):
    write(str(workdir / 'map.yaml'), 'languages: [en]\n')
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'no-heartbeat-timeout' in _codes(issues)


def test_node_key_with_slash(workdir):
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  "cell/motor_a":
    catalog: {workdir / 'a.yaml'}
    base: 1000
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'node-key-has-slash' in _codes(issues)


def test_strict_promotes_warnings_to_errors(workdir):
    # triggers 'no-heartbeat-timeout' only
    write(str(workdir / 'map.yaml'), 'languages: [en]\n')
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, warnings = agg.apply_strict(issues, strict=False)
    assert errors == [] and any(i.code == 'no-heartbeat-timeout' for i in warnings)
    errors_strict, warnings_strict = agg.apply_strict(issues, strict=True)
    assert warnings_strict == []
    assert any(i.code == 'no-heartbeat-timeout' for i in errors_strict)


# ---------------------------------------------------------------------------
# Vendor-YAML robustness
# ---------------------------------------------------------------------------

def test_oversized_file_rejected(workdir):
    path = str(workdir / 'huge.yaml')
    with open(path, 'w') as f:
        f.write('node_name: n\nerror_codes:\n')
        f.write('  X: 1\n# padding\n')
        f.write('#' + ('x' * (agg.MAX_YAML_FILE_BYTES + 100)) + '\n')
    with pytest.raises(ValueError, match='safety limit'):
        agg.load_yaml_file(path)


def test_excessive_node_count_rejected(workdir):
    # A flat list with more entries than the node-count bound, cheap to write.
    path = str(workdir / 'many.yaml')
    count = agg.MAX_YAML_COMPOSED_NODES + 100
    with open(path, 'w') as f:
        f.write('items: [' + ','.join(str(i) for i in range(count)) + ']\n')
    with pytest.raises(yaml.YAMLError, match='node safety limit'):
        agg.load_yaml_file(path)


def test_non_scalar_where_scalar_expected_rejected(workdir):
    path = write(str(workdir / 'n.yaml'), """
node_name: n
error_codes:
  X: 1
descriptions:
  X:
    action: HOLD
    en: { nested: "not a plain string" }
""")
    with pytest.raises(ValueError, match='expected a plain value'):
        agg.parse_node_catalog(path)


def test_package_uri_path_traversal_rejected(workdir, monkeypatch):
    share = str(workdir / 'share' / 'acme_gripper')
    os.makedirs(share, exist_ok=True)
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda pkg: share)
    with pytest.raises(ValueError, match='outside'):
        agg.resolve_package_path('package://acme_gripper/../../../etc/passwd', str(workdir))


def test_package_uri_absolute_relpath_rejected(workdir, monkeypatch):
    # os.path.join discards the share dir entirely when the joined part is
    # absolute, so a double slash ('package://pkg//etc/passwd') would escape
    # without any '..' segment for the lexical traversal check to see.
    share = str(workdir / 'share' / 'acme_gripper')
    os.makedirs(share, exist_ok=True)
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda pkg: share)
    with pytest.raises(ValueError, match='outside'):
        agg.resolve_package_path('package://acme_gripper//etc/passwd', str(workdir))


def test_package_uri_symlinked_share_file_accepted(workdir, monkeypatch):
    # `colcon build --symlink-install` installs share files as symlinks into
    # the source/build tree — resolving to a target outside share/ is the
    # NORMAL case there, not an escape, and must not be rejected.
    share = workdir / 'share' / 'acme_gripper' / 'config'
    os.makedirs(share, exist_ok=True)
    real = write(str(workdir / 'srctree' / 'error_catalog.yaml'), 'node_name: n\n')
    os.symlink(real, str(share / 'error_catalog.yaml'))
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda pkg: str(workdir / 'share' / 'acme_gripper'))
    resolved = agg.resolve_package_path(
        'package://acme_gripper/config/error_catalog.yaml', str(workdir))
    assert os.path.isfile(resolved)
    with open(resolved) as f:
        assert 'node_name' in f.read()


def test_package_uri_resolves_within_share_dir(workdir, monkeypatch):
    share = str(workdir / 'share' / 'acme_gripper')
    os.makedirs(share, exist_ok=True)
    write(os.path.join(share, 'config', 'error_catalog.yaml'), 'node_name: n\n')
    monkeypatch.setattr(
        'ament_index_python.packages.get_package_share_directory',
        lambda pkg: share)
    resolved = agg.resolve_package_path(
        'package://acme_gripper/config/error_catalog.yaml', str(workdir))
    assert resolved == os.path.realpath(os.path.join(share, 'config', 'error_catalog.yaml'))


# ---------------------------------------------------------------------------
# Missing / broken catalog references
# ---------------------------------------------------------------------------

def test_missing_catalog_file_warns_not_crashes(workdir):
    write(str(workdir / 'map.yaml'), """
nodes:
  ghost_node:
    catalog: /nonexistent/path/does_not_exist.yaml
    base: 1000
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'catalog-not-found' in _codes(issues)
    ghost_codes = catalog.nodes.get('ghost_node', {}).get('codes')
    assert 'ghost_node' not in catalog.nodes or not ghost_codes


def test_missing_error_map_raises_aggregation_error(workdir):
    with pytest.raises(agg.AggregationError):
        agg.aggregate(str(workdir / 'does_not_exist.yaml'))


# ---------------------------------------------------------------------------
# CLI end-to-end (subprocess) — output files + exit codes
# ---------------------------------------------------------------------------

def test_cli_list_inputs(workdir):
    _minimal_node_yaml(str(workdir / 'motor.yaml'), 'n')
    map_path = write(str(workdir / 'map.yaml'), f"""
nodes:
  n:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
""")
    result = subprocess.run(
        [sys.executable, _SCRIPT, '--map', map_path, '--list-inputs'],
        capture_output=True, text=True, check=True)
    lines = result.stdout.strip().splitlines()
    assert str(map_path) in lines
    assert str(workdir / 'motor.yaml') in lines


def test_cli_full_run_writes_all_default_formats(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: n
error_codes:
  X: 1
descriptions:
  X: { action: HOLD, severity: ERROR, en: "x fault" }
""")
    map_path = write(str(workdir / 'map.yaml'), f"""
languages: [en]
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "hb lost" }}
nodes:
  n:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
""")
    out_dir = str(workdir / 'out')
    result = subprocess.run(
        [sys.executable, _SCRIPT, '--map', map_path, '--output-dir', out_dir],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert os.path.isfile(os.path.join(out_dir, 'machine_error_catalog.yaml'))
    assert os.path.isfile(os.path.join(out_dir, 'machine_errors.json'))
    assert os.path.isfile(os.path.join(out_dir, 'machine_errors.md'))

    with open(os.path.join(out_dir, 'machine_errors.json')) as f:
        doc = json.load(f)
    globals_seen = {row['global_code'] for row in doc['faults']}
    assert 1 in globals_seen  # reserved heartbeat_timeout
    assert 1001 in globals_seen  # node n code 1 -> base 1000 + 1


def test_cli_strict_fails_build_on_warning(workdir):
    map_path = write(str(workdir / 'map.yaml'), 'languages: [en]\n')  # W8 only
    out_dir = str(workdir / 'out')
    result = subprocess.run(
        [sys.executable, _SCRIPT, '--map', map_path, '--output-dir', out_dir, '--strict'],
        capture_output=True, text=True)
    assert result.returncode == 1
    assert 'heartbeat_timeout' in result.stderr


def test_cli_depfile_lists_map_and_node_catalogs(workdir):
    catalog_path = _minimal_node_yaml(str(workdir / 'motor.yaml'), 'n')
    map_path = write(str(workdir / 'map.yaml'), f"""
nodes:
  n:
    catalog: {catalog_path}
    base: 1000
""")
    out_dir = str(workdir / 'out')
    depfile = str(workdir / 'out.d')
    result = subprocess.run(
        [sys.executable, _SCRIPT, '--map', map_path, '--output-dir', out_dir,
         '--depfile', depfile],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    with open(depfile) as f:
        content = f.read()
    assert str(map_path) in content
    assert str(catalog_path) in content
    assert content.strip().startswith(os.path.join(out_dir, 'machine_error_catalog.yaml'))


# ---------------------------------------------------------------------------
# Regression tests for the adversarial-review findings
# ---------------------------------------------------------------------------

def test_languages_defaults_to_union_of_locales_found(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: motor
error_codes:
  X: 1
descriptions:
  X: { action: HOLD, en: "English", nl: "Dutch", fr: "French" }
""")
    write(str(workdir / 'map.yaml'), f"""
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "hb", nl: "hb nl" }}
nodes:
  motor:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert errors == []
    assert catalog.languages == ['en', 'fr', 'nl']


def test_explicit_languages_not_overridden_by_union(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en]
reserved:
  heartbeat_timeout: { global: 1, action: ABORT, en: "hb", nl: "hb nl (extra, not declared)" }
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert catalog.languages == ['en']
    assert any(i.code == 'unconfigured-locale' for i in issues)


def test_orphan_override_key_warns(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: motor
error_codes:
  X: 1
descriptions:
  X: { action: HOLD, en: "x" }
""")
    write(str(workdir / 'map.yaml'), f"""
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "hb" }}
nodes:
  motor:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
    overrides:
      999:
        en: "this code does not exist in motor.yaml"
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert any(i.code == 'orphan-override' for i in issues)


def test_orphan_remap_key_warns_and_does_not_silently_misassign(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: motor
error_codes:
  X: 5
descriptions:
  X: { action: HOLD, en: "x" }
""")
    write(str(workdir / 'map.yaml'), f"""
reserved:
  heartbeat_timeout: {{ global: 1, action: ABORT, en: "hb" }}
nodes:
  motor:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
    remap: {{ 55: 5090 }}
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert any(i.code == 'orphan-remap' for i in issues)
    # Code 5 still gets the ordinary base+local global — the dangling remap
    # key must not silently corrupt the real code's assignment.
    assert catalog.nodes['motor']['codes'][5]['global'] == 1005


def test_remap_key_must_be_positive(workdir):
    write(str(workdir / 'motor.yaml'), """
node_name: motor
error_codes:
  X: 5
descriptions:
  X: { action: HOLD, en: "x" }
""")
    write(str(workdir / 'map.yaml'), f"""
nodes:
  motor:
    catalog: {workdir / 'motor.yaml'}
    base: 1000
    remap: {{ -5: 5090 }}
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'local-code-out-of-range' in _codes(issues)


def test_malformed_base_window_remap_report_clean_error_not_crash(workdir):
    _minimal_node_yaml(str(workdir / 'motor.yaml'), 'motor')
    for bad_field, bad_yaml in [
        ('base', 'base: "not_a_number"'),
        ('base', 'base: [1000, 2000]'),
        ('remap', 'remap: { "1.0": 5090 }'),
    ]:
        map_path = write(str(workdir / f'map_{bad_field}_{hash(bad_yaml)}.yaml'), f"""
nodes:
  motor:
    catalog: {workdir / 'motor.yaml'}
    {bad_yaml}
""")
        catalog, issues = agg.aggregate(map_path)
        assert 'motor' not in catalog.nodes
        assert any(i.code == 'node-malformed' for i in issues)


def test_cli_survives_wrong_shape_yaml_without_traceback(workdir):
    # A null descriptions block must not raise an unhandled AttributeError.
    map_path = write(str(workdir / 'map.yaml'), """
instances:
  - 1
  - 2
""")
    result = subprocess.run(
        [sys.executable, _SCRIPT, '--map', map_path, '--output-dir', str(workdir / 'out')],
        capture_output=True, text=True)
    assert result.returncode == 1
    assert 'Traceback' not in result.stderr
    assert 'error:' in result.stderr


def test_reserved_instanced_key_does_not_crash_and_is_flagged(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  heartbeat_timeout: { global: 1, action: ABORT, en: "hb lost", instanced: true }
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert any(i.code == 'reserved-instanced-key' for i in issues)
    # The bogus key must not leak into descriptions as a phantom locale.
    assert 'instanced' not in catalog.reserved['heartbeat_timeout']['descriptions']


def test_window_overlap_does_not_false_positive_on_degenerate_window(workdir):
    # 'other's window [1050, 1150) numerically brackets motor's degenerate
    # (empty) window at 1100 -- the old, unguarded overlap test flagged this
    # as an "overlap" even though an empty interval can never really collide
    # with anything. Both bases are kept >= 100 so the (unrelated, correct)
    # reserved-band check doesn't also fire and confound the assertion.
    _minimal_node_yaml(str(workdir / 'a.yaml'), 'a')
    _minimal_node_yaml(str(workdir / 'b.yaml'), 'b')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  motor:
    catalog: {workdir / 'a.yaml'}
    base: 1100
    window: 0
  other:
    catalog: {workdir / 'b.yaml'}
    base: 1050
    window: 100
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'window-overlap' not in _codes(issues)
    # The degenerate window itself is still flagged, just not as a false
    # "overlap" with an unrelated node.
    assert 'value-out-of-range' in _codes(issues)


def test_unrecognized_action_warns(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 1, action: ABROT, en: "typo'd action" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert any(i.code == 'unrecognized-action' for i in issues)


def test_collect_dependencies_skips_nonexistent_resolved_file(workdir):
    write(str(workdir / 'map.yaml'), """
nodes:
  motor:
    catalog: /nonexistent/path/does_not_exist.yaml
    base: 1000
""")
    deps = agg.collect_dependencies(str(workdir / 'map.yaml'))
    assert not any('does_not_exist' in d for d in deps)


def test_collect_dependencies_dedupes_shared_catalog(workdir):
    catalog_path = _minimal_node_yaml(str(workdir / 'shared.yaml'), 'shared')
    write(str(workdir / 'map.yaml'), f"""
nodes:
  a:
    catalog: {catalog_path}
    base: 1000
  b:
    catalog: {catalog_path}
    base: 2000
""")
    deps = agg.collect_dependencies(str(workdir / 'map.yaml'))
    assert deps.count(catalog_path) == 1


def test_write_depfile_escapes_make_special_characters(workdir):
    assert agg._escape_make_path('a:b') == r'a\:b'
    assert agg._escape_make_path('a$b') == 'a$$b'
    assert agg._escape_make_path('a#b') == r'a\#b'
    assert agg._escape_make_path('a b') == r'a\ b'


def test_strict_not_required_for_structural_failures(workdir):
    # catalog-not-found, catalog-parse-error, no-catalog, and node-malformed
    # must fail the build by default -- a broken node reference must not
    # silently produce an incomplete catalog with colcon build reporting
    # success (the whole point of this being a build-time tool).
    write(str(workdir / 'map.yaml'), """
nodes:
  ghost:
    catalog: /nonexistent/path/does_not_exist.yaml
    base: 1000
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    errors, _ = agg.apply_strict(issues, strict=False)
    assert any(i.code == 'catalog-not-found' for i in errors)


# ---------------------------------------------------------------------------
# Category taxonomy, schema version, locale-tag normalization
# ---------------------------------------------------------------------------

def test_unrecognized_category_warns(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 1, action: ABORT, category: Electrical, en: "x" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert any(i.code == 'unrecognized-category' for i in issues)


def test_known_category_does_not_warn(workdir):
    write(str(workdir / 'map.yaml'), """
reserved:
  fault_a: { global: 1, action: ABORT, category: electrical, en: "x" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert not any(i.code == 'unrecognized-category' for i in issues)


def test_write_machine_yaml_includes_schema_version(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en]
reserved:
  heartbeat_timeout: { global: 1, action: ABORT, en: "hb lost" }
""")
    catalog, issues = agg.aggregate(str(workdir / 'map.yaml'))
    provenance = agg._provenance(str(workdir / 'map.yaml'), catalog)
    out_path = str(workdir / 'out' / 'machine_error_catalog.yaml')
    agg.write_machine_yaml(catalog, provenance, out_path)
    with open(out_path) as f:
        doc = yaml.safe_load(f)
    assert doc['schema_version'] == agg._schema.SCHEMA_VERSION


def test_locale_case_variant_does_not_warn_unconfigured(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en]
reserved:
  fault_a: { global: 1, action: ABORT, EN: "upper-case locale key" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert not any(i.code == 'unconfigured-locale' for i in issues)


def test_locale_region_variant_satisfies_language_completeness(workdir):
    write(str(workdir / 'map.yaml'), """
languages: [en]
reserved:
  fault_a: { global: 1, action: ABORT, en-US: "region-tagged locale key" }
""")
    _, issues = agg.aggregate(str(workdir / 'map.yaml'))
    assert 'description-missing-locale' not in _codes(issues)
    assert not any(i.code == 'unconfigured-locale' for i in issues)
