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
Offline error-catalog aggregation tool.

Reads a bringup package's error_map.yaml, resolves and parses every
referenced per-node error_catalog.yaml, assigns each node-local code a
globally-unique fault number, merges integrator recovery-text overrides, and
emits the manager's runtime artifact (machine_error_catalog.yaml) plus a
static HMI lookup (machine_errors.json) and a human-readable fault sheet
(machine_errors.md).

This is the single parser for per-node error_catalog.yaml — the C++ library
(error_catalog.hpp/.cpp) only ever loads the already-aggregated
machine_error_catalog.yaml at runtime; there is no C++ node-level parser to
keep in sync with.

Usage:
  aggregate_error_catalog.py --map error_map.yaml --output-dir <dir>
      [--formats yaml,json,md] [--strict] [--check-instances id1,id2,...]
  aggregate_error_catalog.py --map error_map.yaml --list-inputs
"""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
import sys

import yaml

TOOL_VERSION = '1.0.0'

# ---------------------------------------------------------------------------
# Schema keys — imported from generate_error_codes_header.py (the single
# source of truth for section/field names shared with the C++ loader), not
# re-declared here, so the two YAML readers this tool touches (this tool's own
# per-node/error_map parsing, and error_catalog.cpp's machine-catalog loader)
# can't independently drift on the same field names.
# ---------------------------------------------------------------------------


def _load_schema_module():
    """Locate and import generate_error_codes_header.py.

    Tries a source-tree-relative sibling path first (this script and the
    codegen script are both under packml_ros/ in the source tree), then falls
    back to the installed package's share/cmake directory (this script's own
    installed location, lib/<pkg>/, is a sibling install-space directory of
    share/<pkg>/cmake/ under the same package prefix).
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', 'cmake', 'generate_error_codes_header.py'),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        cmake_dir = os.path.join(get_package_share_directory('packml_ros'), 'cmake')
        candidates.append(os.path.join(cmake_dir, 'generate_error_codes_header.py'))
    except Exception:
        pass

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            spec = importlib.util.spec_from_file_location(
                'generate_error_codes_header', candidate)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError(
        'Could not locate generate_error_codes_header.py (tried: '
        + ', '.join(candidates) + ') — packml_ros installation looks broken')


_schema = _load_schema_module()

# --- error_map.yaml-only keys (not shared with the C++ loader, which never
# reads error_map.yaml — only this tool does) ---
CATALOG_KEY = 'catalog'
BASE_KEY = 'base'
WINDOW_KEY = 'window'
REMAP_KEY = 'remap'
OVERRIDES_KEY = 'overrides'
DEFAULT_WINDOW = 1000
RESERVED_BAND = range(1, 100)  # conventional 1-99 reserved band (W7)
INT32_MAX = 2 ** 31 - 1
KNOWN_SEVERITIES = ('CRITICAL', 'ERROR', 'WARNING', 'INFO')
KNOWN_ACTIONS = ('NONE', 'WARN', 'HOLD', 'SUSPEND', 'ABORT')
MAX_DESCRIPTION_CHARS = 80  # W2: PackTags Message field limit

# Must match packml_ros::kManagerNodeName (error_catalog.hpp) — the sentinel
# "owning node" label for manager-synthesized reserved faults in the flattened
# JSON/MD output rows. Not schema-shared like the YAML section keys above:
# this is an output-only label (never read back by the C++ loader), so a
# single named constant with this cross-reference is proportionate.
MANAGER_NODE_NAME = 'manager'

# ---------------------------------------------------------------------------
# Vendor-YAML robustness — the per-node catalogs this tool reads are, by
# design, authored by third-party vendor packages this repo does not control.
# ---------------------------------------------------------------------------

MAX_YAML_FILE_BYTES = 1_000_000  # 1 MB; real catalogs are a few KB at most
MAX_YAML_COMPOSED_NODES = 20_000  # bounds a compact anchor/alias bomb


class _BoundedSafeLoader(yaml.SafeLoader):
    """SafeLoader that aborts once too many nodes have been composed.

    A YAML "billion laughs" style anchor/alias bomb keeps the ON-DISK file
    and the composed node GRAPH small (PyYAML resolves an alias to the
    already-composed node object by reference, it does not re-expand it) —
    the danger is in code that later walks the LOGICAL (fully expanded) tree,
    e.g. naive recursive serialization. This tool never does an open-ended
    recursive walk (every value is read from a specific, schema-fixed key
    path and validated as a plain scalar — see _require_scalar), so the node
    count itself is already a reasonable, cheap proxy for "this file is
    absurdly large/complex" regardless of how it got that way.
    """

    def compose_node(self, parent, index):
        count = getattr(self, '_node_count', 0) + 1
        self._node_count = count
        if count > MAX_YAML_COMPOSED_NODES:
            raise yaml.YAMLError(
                f'YAML document exceeds the {MAX_YAML_COMPOSED_NODES}-node safety '
                'limit (possible anchor/alias expansion or a corrupted file)')
        return super().compose_node(parent, index)


def load_yaml_file(path):
    """Load a YAML file with resource bounds for untrusted vendor input.

    A file-size cap is checked before reading, and a node-count cap during
    parsing (see _BoundedSafeLoader).
    """
    size = os.path.getsize(path)
    if size > MAX_YAML_FILE_BYTES:
        raise ValueError(
            f"'{path}' is {size} bytes, over the {MAX_YAML_FILE_BYTES}-byte "
            'vendor-catalog safety limit')
    with open(path, 'r') as f:
        return yaml.load(f, Loader=_BoundedSafeLoader) or {}


def _require_scalar(value, context):
    """Reject a non-scalar (list/dict) where a plain value is expected.

    This is primarily a correctness check (a mis-indented YAML file silently
    producing a nested structure where a string/int/bool was expected is a
    confusing failure to debug) but is also the key defense against a
    maliciously-aliased nested structure disguised as a scalar: Python's
    str()/repr() on such a structure is NOT identity-aware and can produce
    an astronomically long string for a deeply-aliased non-cyclic DAG, even
    though the parsed object graph itself stayed small. Rejecting non-scalars
    outright at every leaf this tool reads means that string conversion is
    never reached on attacker-controlled nested data.
    """
    if isinstance(value, (dict, list)):
        raise ValueError(f'{context}: expected a plain value, got a {type(value).__name__}')
    return value


def resolve_package_path(spec, base_dir):
    """Resolve a `catalog:` path spec.

    Accepts package://<pkg>/<relpath>, an absolute path, or a path relative
    to the error_map.yaml's own directory. A package:// URI is contained to
    the resolved package's own share directory — a `../` escape (e.g.
    package://foo/../../../etc/passwd) is rejected, since the package name in
    the URI is meant to be an authorization boundary (this package's own
    install, nothing else).
    """
    if spec.startswith('package://'):
        rest = spec[len('package://'):]
        if '/' not in rest:
            raise ValueError(
                f"malformed package:// URI (no path after the package name): '{spec}'")
        pkg, relpath = rest.split('/', 1)
        from ament_index_python.packages import get_package_share_directory
        share_dir = os.path.realpath(get_package_share_directory(pkg))
        resolved = os.path.realpath(os.path.join(share_dir, relpath))
        if os.path.commonpath([resolved, share_dir]) != share_dir:
            raise ValueError(
                f"package:// URI '{spec}' resolves outside package '{pkg}'’s share "
                f'directory — rejected')
        return resolved
    if os.path.isabs(spec):
        return spec
    return os.path.realpath(os.path.join(base_dir, spec))


# ---------------------------------------------------------------------------
# Per-node catalog parsing — the sole parser now that the C++
# load_node_catalog_from_yaml has been deleted as production-dead.
# ---------------------------------------------------------------------------

_INSTANCE_PLACEHOLDER = '{instance}'


class LintIssue:
    def __init__(self, code, message):
        self.code = code
        self.message = message

    def __str__(self):
        return f'[{self.code}] {self.message}'


class AggregationError(Exception):
    """Raised for a hard failure (bad file, or a promoted/error-level lint)."""


def parse_node_catalog(path):
    """Parse one per-node error_catalog.yaml.

    Returns (entries, issues), where entries is a dict of local_code -> entry
    dict (code, name, action, severity, category, instanced, descriptions),
    mirroring the semantics of the deleted C++ load_node_catalog_from_yaml:
    code 0 rejected, duplicate local value warns (last wins), codes without a
    description are excluded, orphan descriptions warn, negative locals
    rejected (E2 covers >= window separately; this rejects < 1 outright, same
    as the as-built loader's reserved-0 rule extended to all non-positive
    values).
    """
    issues = []
    data = load_yaml_file(path)

    error_codes = data.get(_schema.ERROR_CODES_KEY, {}) or {}
    descriptions_block = data.get(_schema.DESCRIPTIONS_KEY, {}) or {}

    code_by_name = {}
    seen_codes = set()
    for name, code in error_codes.items():
        name = _require_scalar(name, f'{path}: error_codes key')
        code = int(_require_scalar(code, f"{path}: error_codes['{name}']"))
        if code < 1:
            issues.append(LintIssue(
                'reserved-code', f"'{path}': error code '{name}' uses non-positive "
                f'value {code} — entry rejected'))
            continue
        if code in seen_codes:
            issues.append(LintIssue(
                'duplicate-local', f"'{path}': duplicate error code value {code} "
                f"('{name}') — last wins"))
        seen_codes.add(code)
        code_by_name[name] = code

    entries = {}
    described_names = set()
    for name, block in descriptions_block.items():
        described_names.add(name)
        if name not in code_by_name:
            issues.append(LintIssue(
                'orphan-description',
                f"'{path}': description '{name}' has no matching entry in error_codes "
                '— ignored'))
            continue

        entry = {
            'code': code_by_name[name],
            'name': name,
            'action': 'NONE',
            'severity': '',
            'category': '',
            'instanced': False,
            'descriptions': {},
        }
        for key, value in block.items():
            if key == _schema.ACTION_KEY:
                entry['action'] = _require_scalar(value, f"{path}: '{name}'.action")
            elif key == _schema.SEVERITY_KEY:
                entry['severity'] = _require_scalar(value, f"{path}: '{name}'.severity")
            elif key == _schema.CATEGORY_KEY:
                entry['category'] = _require_scalar(value, f"{path}: '{name}'.category")
            elif key == _schema.INSTANCED_KEY:
                entry['instanced'] = bool(_require_scalar(value, f"{path}: '{name}'.instanced"))
            else:
                entry['descriptions'][key] = _require_scalar(value, f"{path}: '{name}'.{key}")

        if not entry['descriptions']:
            issues.append(LintIssue(
                'no-description', f"'{path}': error code '{name}' has no locale descriptions"))

        for locale, text in entry['descriptions'].items():
            has_placeholder = _INSTANCE_PLACEHOLDER in text
            if entry['instanced'] and not has_placeholder:
                issues.append(LintIssue(
                    'E5', f"'{path}': instanced error code '{name}' description [{locale}] "
                    f'has no "{_INSTANCE_PLACEHOLDER}" placeholder'))
            elif not entry['instanced'] and has_placeholder:
                issues.append(LintIssue(
                    'E5', f"'{path}': non-instanced error code '{name}' description "
                    f'[{locale}] contains a "{_INSTANCE_PLACEHOLDER}" placeholder'))

        entries[entry['code']] = entry

    for name, code in code_by_name.items():
        if name not in described_names:
            issues.append(LintIssue(
                'missing-description',
                f"'{path}': error code '{name}' ({code}) has no description block"))

    return entries, issues


# ---------------------------------------------------------------------------
# error_map.yaml parsing + aggregation
# ---------------------------------------------------------------------------


class AggregatedCatalog:
    def __init__(self):
        self.languages = []
        self.reserved = {}   # name -> entry dict (+ 'global')
        self.instances = {}  # id -> {locale: label}
        # node_name -> {'base': int, 'window': int, 'codes': {local: entry+global}}
        self.nodes = {}
        self.provenance_sources = []  # list of (path, sha256)


def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def aggregate(map_path, check_instances=None):
    """Parse error_map.yaml and produce the aggregated, linted catalog.

    Resolves and parses every referenced node catalog, assigns global fault
    numbers, merges integrator recovery-text overrides, and lints the
    result. Returns (AggregatedCatalog, list[LintIssue]). Raises
    AggregationError for a hard failure (unreadable file, or a fatal
    structural problem).
    """
    issues = []
    map_dir = os.path.dirname(os.path.abspath(map_path))
    try:
        root = load_yaml_file(map_path)
    except (OSError, yaml.YAMLError, ValueError) as e:
        raise AggregationError(f"failed to load error map '{map_path}': {e}") from e

    catalog = AggregatedCatalog()
    languages_declared = _schema.LANGUAGES_KEY in root
    catalog.languages = list(root.get(_schema.LANGUAGES_KEY, []) or [])

    # --- instances: id -> {locale: label} ---
    for inst_id, labels in (root.get(_schema.INSTANCES_KEY, {}) or {}).items():
        catalog.instances[inst_id] = {
            locale: _require_scalar(label, f'instances.{inst_id}.{locale}')
            for locale, label in (labels or {}).items()
        }
    if check_instances:
        for inst_id in check_instances:
            if inst_id not in catalog.instances:
                issues.append(LintIssue(
                    'W4', f"instance id '{inst_id}' (from --check-instances) has no label "
                    "in 'instances:'"))

    # --- reserved: name -> {global, action, severity, category, locales...} ---
    reserved_globals = {}
    for name, block in (root.get(_schema.RESERVED_KEY, {}) or {}).items():
        block = block or {}
        if _schema.GLOBAL_KEY not in block:
            issues.append(LintIssue(
                'reserved-no-global', f"reserved '{name}' has no '{_schema.GLOBAL_KEY}' "
                'number — skipped'))
            continue
        global_num = int(_require_scalar(block[_schema.GLOBAL_KEY], f'reserved.{name}.global'))
        entry = {
            'global': global_num, 'name': name, 'action': 'NONE', 'severity': '',
            'category': '', 'instanced': False, 'descriptions': {},
        }
        for key, value in block.items():
            if key == _schema.GLOBAL_KEY:
                continue
            elif key == _schema.ACTION_KEY:
                action_str = _require_scalar(value, f'reserved.{name}.action')
                if action_str not in KNOWN_ACTIONS:
                    issues.append(LintIssue(
                        'unrecognized-action', f"reserved '{name}' has action '{action_str}' "
                        f'outside {KNOWN_ACTIONS}'))
                entry['action'] = action_str
            elif key == _schema.SEVERITY_KEY:
                entry['severity'] = _require_scalar(value, f'reserved.{name}.severity')
            elif key == _schema.CATEGORY_KEY:
                entry['category'] = _require_scalar(value, f'reserved.{name}.category')
            elif key == _schema.INSTANCED_KEY:
                # Reserved entries auto-derive 'instanced' from their description
                # text below — an explicit key here isn't read by the C++ loader
                # either, so keep it out of descriptions rather than letting it
                # become a phantom locale (and crash the any(...) check below).
                issues.append(LintIssue(
                    'reserved-instanced-key',
                    f"reserved '{name}' has an explicit '{_schema.INSTANCED_KEY}' key — "
                    "reserved entries auto-derive 'instanced' from description "
                    'placeholders instead; this key is ignored'))
            else:
                entry['descriptions'][key] = _require_scalar(value, f'reserved.{name}.{key}')
        entry['instanced'] = any(
            isinstance(t, str) and _INSTANCE_PLACEHOLDER in t
            for t in entry['descriptions'].values())

        if global_num in reserved_globals:
            issues.append(LintIssue(
                'E4', f"duplicate global fault number {global_num} (reserved '{name}' vs "
                f"'{reserved_globals[global_num]}')"))
        reserved_globals[global_num] = name
        if global_num not in RESERVED_BAND:
            issues.append(LintIssue(
                'W7', f"reserved '{name}' has global {global_num} outside the "
                f'conventional 1-99 reserved band'))
        if not (0 < global_num <= INT32_MAX):
            issues.append(LintIssue(
                'E6', f"reserved '{name}' global {global_num} is outside [1, {INT32_MAX}]"))
        catalog.reserved[name] = entry
        _lint_entry_common(entry, f"reserved '{name}'", issues)

    if 'heartbeat_timeout' not in catalog.reserved:
        issues.append(LintIssue('W8', "no 'heartbeat_timeout' reserved entry defined"))

    # --- nodes: node_name -> {catalog, base, window, remap, overrides} ---
    all_globals = dict(reserved_globals)  # global -> owner label, for E3/E4 cross-checks
    node_windows = []  # (node_name, base, base+window) for E1

    for node_name, node_spec in (root.get(_schema.NODES_KEY, {}) or {}).items():
        if '/' in node_name:
            issues.append(LintIssue(
                'W9', f"node key '{node_name}' contains '/' (a ROS namespace) — node "
                'identity must be flat, see error_map.yaml "Node identity"'))
        node_spec = node_spec or {}
        try:
            base = int(_require_scalar(node_spec.get(BASE_KEY, 0), f'nodes.{node_name}.base'))
            window = int(_require_scalar(
                node_spec.get(WINDOW_KEY, DEFAULT_WINDOW), f'nodes.{node_name}.window'))
            remap = {
                int(_require_scalar(k, f'nodes.{node_name}.remap key')):
                int(_require_scalar(v, f'nodes.{node_name}.remap[{k}]'))
                for k, v in (node_spec.get(REMAP_KEY, {}) or {}).items()
            }
        except (ValueError, TypeError) as e:
            issues.append(LintIssue(
                'node-malformed', f"node '{node_name}': {e} — node skipped"))
            continue
        overrides = node_spec.get(OVERRIDES_KEY, {}) or {}

        # A non-positive window can never contain an assignable local code (by
        # base+local or by an own-window remap) — flag it at the source instead
        # of letting it manifest as a spray of misleading E1/E2 messages below.
        window_usable = window > 0
        if not window_usable:
            issues.append(LintIssue(
                'E6', f"node '{node_name}' has a non-positive window ({window}) — no code "
                'could ever be assigned to it'))
        # E2's bounds apply to remap keys too: same positivity rule as an
        # ordinary local code (remap keys are exempt from the upper "< window"
        # bound by design — that's what lets a wild vendor code escape it).
        for remap_local in remap:
            if remap_local <= 0:
                issues.append(LintIssue(
                    'E2', f"node '{node_name}' remap key {remap_local} is non-positive — "
                    'the same positivity bound as an ordinary local code applies'))

        if base + window - 1 > INT32_MAX or base < 0:
            issues.append(LintIssue(
                'E6', f"node '{node_name}' window [{base}, {base + window}) exceeds "
                f'[1, {INT32_MAX}]'))

        if window_usable:
            for other_name, other_lo, other_hi in node_windows:
                if other_hi > other_lo and base < other_hi and other_lo < base + window:
                    issues.append(LintIssue(
                        'E1', f"node '{node_name}' window [{base}, {base + window}) overlaps "
                        f"node '{other_name}' window [{other_lo}, {other_hi})"))
            for reserved_g in reserved_globals:
                if base <= reserved_g < base + window:
                    issues.append(LintIssue(
                        'E1', f"node '{node_name}' window [{base}, {base + window}) overlaps "
                        f'reserved value {reserved_g}'))
            if base < 100:
                # Overlaps the conventional reserved band even if no reserved
                # value happens to be assigned there yet.
                issues.append(LintIssue(
                    'E1', f"node '{node_name}' window [{base}, {base + window}) overlaps "
                    'the conventional 1-99 reserved band'))
        node_windows.append((node_name, base, base + window))

        if CATALOG_KEY not in node_spec:
            issues.append(LintIssue(
                'no-catalog', f"node '{node_name}' has no '{CATALOG_KEY}' entry — skipped"))
            continue
        try:
            catalog_path = resolve_package_path(node_spec[CATALOG_KEY], map_dir)
        except Exception as e:
            issues.append(LintIssue(
                'catalog-not-found', f"node '{node_name}': {e}"))
            continue
        if not os.path.isfile(catalog_path):
            issues.append(LintIssue(
                'catalog-not-found',
                f"node '{node_name}': catalog file not found: '{catalog_path}' "
                '(is the owning package installed and declared as a <depend> of the '
                'bringup package?)'))
            continue

        try:
            node_entries, node_issues = parse_node_catalog(catalog_path)
        except (OSError, yaml.YAMLError, ValueError, AttributeError, TypeError, KeyError) as e:
            issues.append(LintIssue(
                'catalog-parse-error', f"node '{node_name}': failed to parse "
                f"'{catalog_path}': {e}"))
            continue
        issues.extend(node_issues)
        # Multiple node identities may legitimately share one catalog file
        # (see error_map.yaml's "Node identity" note) — record it once.
        if catalog_path not in {p for p, _ in catalog.provenance_sources}:
            catalog.provenance_sources.append((catalog_path, _sha256_file(catalog_path)))

        if not node_entries:
            issues.append(LintIssue('W3', f"node '{node_name}' catalog has zero usable codes"))

        codes_out = {}
        for local, entry in node_entries.items():
            if local in remap:
                global_num = remap[local]
            elif window_usable and 0 < local < window:
                global_num = base + local
            else:
                issues.append(LintIssue(
                    'E2', f"node '{node_name}' local code {local} is outside its window "
                    f'[1, {window}) and has no remap entry — widen the window or add a remap'))
                continue

            if global_num in all_globals:
                issues.append(LintIssue(
                    'E4', f'duplicate global fault number {global_num} (node '
                    f"'{node_name}' code {local} vs '{all_globals[global_num]}')"))
            all_globals[global_num] = f'{node_name}:{local}'

            if local in remap:
                # E3: a remap target must land in the node's OWN window (permitted —
                # that's the intended use, e.g. renumbering a wild vendor code into your
                # own subsystem block) or must not collide with another node's window /
                # a reserved value. Collision with another resulting global is E4, above.
                in_own_window = window_usable and base <= global_num < base + window
                if not in_own_window:
                    for other_name, other_lo, other_hi in node_windows[:-1]:
                        if other_lo <= global_num < other_hi:
                            issues.append(LintIssue(
                                'E3', f"node '{node_name}' remap {local} -> {global_num} "
                                f"lands inside node '{other_name}'’s window"))
                    if global_num in reserved_globals:
                        issues.append(LintIssue(
                            'E3', f"node '{node_name}' remap {local} -> {global_num} "
                            'collides with a reserved value'))
                if not (0 < global_num <= INT32_MAX):
                    issues.append(LintIssue(
                        'E6', f"node '{node_name}' remap {local} -> {global_num} is "
                        f'outside [1, {INT32_MAX}]'))

            merged = dict(entry)
            merged['global'] = global_num
            override = overrides.get(local) or overrides.get(str(local))
            if override:
                merged_descriptions = dict(entry['descriptions'])
                for locale, text in override.items():
                    merged_descriptions[locale] = _require_scalar(
                        text, f"node '{node_name}' overrides.{local}.{locale}")
                merged['descriptions'] = merged_descriptions
            codes_out[local] = merged
            _lint_entry_common(merged, f"node '{node_name}' code {local}", issues)

        # An override keyed by a local code that never matched any parsed
        # vendor entry (stale, or mistyped, e.g. after a vendor renumbering)
        # would otherwise silently vanish with no trace it was ever written.
        consumed_override_keys = set()
        for local in codes_out:
            consumed_override_keys.update({local, str(local)} & set(overrides))
        for stale_key in overrides:
            if stale_key not in consumed_override_keys:
                issues.append(LintIssue(
                    'orphan-override',
                    f"node '{node_name}' overrides key {stale_key!r} does not match any "
                    f"local code in '{catalog_path}' — override text is silently ignored "
                    '(stale or mistyped key?)'))

        # A remap key that never matched any parsed vendor code is similarly
        # dead — worse, a transposed-digit typo of a real code silently falls
        # through to the DEFAULT base+local global instead of the intended
        # remap target, with zero warning.
        for remap_local in remap:
            if remap_local not in node_entries:
                issues.append(LintIssue(
                    'orphan-remap',
                    f"node '{node_name}' remap key {remap_local} does not match any local "
                    f"code in '{catalog_path}' — remap entry has no effect"))

        catalog.nodes[node_name] = {'base': base, 'window': window, 'codes': codes_out}

    # languages: is documented as optional, defaulting to the union of locales
    # actually found — computed here, now that every entry (reserved + every
    # node's merged/overridden codes) is known, rather than left empty.
    if not languages_declared:
        found = set()
        for entry in catalog.reserved.values():
            found.update(entry.get('descriptions', {}))
        for node in catalog.nodes.values():
            for entry in node['codes'].values():
                found.update(entry.get('descriptions', {}))
        catalog.languages = (['en'] if 'en' in found else []) + sorted(found - {'en'})

    # Locale-completeness/consistency lints depend on the final languages list
    # above, so they run in one pass after it's settled (not inline during the
    # entry-building loops, where an auto-computed default wouldn't exist yet).
    for name, entry in catalog.reserved.items():
        _lint_entry_languages(entry, f"reserved '{name}'", catalog, issues)
    for node_name, node in catalog.nodes.items():
        for local, entry in node['codes'].items():
            _lint_entry_languages(entry, f"node '{node_name}' code {local}", catalog, issues)

    return catalog, issues


def _lint_entry_common(entry, label, issues):
    """Lint rules independent of the final languages list.

    W2 (description length) and the action/severity vocabulary checks
    (unrecognized-action, W6).
    """
    if entry.get('severity') and entry['severity'] not in KNOWN_SEVERITIES:
        issues.append(LintIssue(
            'W6', f"{label} has severity '{entry['severity']}' outside "
            f'{KNOWN_SEVERITIES}'))
    if entry.get('action') and entry['action'] not in KNOWN_ACTIONS:
        issues.append(LintIssue(
            'unrecognized-action', f"{label} has action '{entry['action']}' outside "
            f'{KNOWN_ACTIONS}'))
    for locale, text in entry.get('descriptions', {}).items():
        if len(text) > MAX_DESCRIPTION_CHARS:
            issues.append(LintIssue(
                'W2', f'{label} description [{locale}] is {len(text)} chars, over the '
                f'{MAX_DESCRIPTION_CHARS}-char PackTags Message limit'))


def _lint_entry_languages(entry, label, catalog, issues):
    """Lint rules depending on the final (possibly auto-computed) languages list.

    W5 (a declared language missing from this entry) and its mirror — a
    description key that ISN'T a configured language, almost always a
    typo'd field name (e.g. 'sevrity' for 'severity') rather than a real,
    unconfigured locale. Mirrors error_catalog.cpp's identical cross-check
    for the already-aggregated catalog (see its v2 delta) — this is the
    Python-side half for the actual vendor-authored YAML this tool parses.
    """
    descriptions = entry.get('descriptions', {})
    for lang in catalog.languages:
        if lang not in descriptions:
            issues.append(LintIssue(
                'W5', f"{label} has no '{lang}' description (declared in 'languages:')"))
    if catalog.languages:
        for locale in descriptions:
            if locale not in catalog.languages:
                issues.append(LintIssue(
                    'unconfigured-locale',
                    f"{label} has description key '{locale}' which is not in the "
                    "configured 'languages' list — check for a typo of a field name "
                    '(action/severity/category/instanced) or add the locale to '
                    "'languages:'"))


def apply_strict(issues, strict):
    """Promote warnings to errors in --strict mode.

    Returns (errors, warnings) split from the combined issue list. A handful
    of non-E-numbered codes are unconditionally fatal (not just under
    --strict) because they mean a chunk of the catalog silently failed to
    aggregate at all — treating that as a mere warning would let `colcon
    build` report success while shipping an incomplete/corrupt runtime
    catalog.
    """
    error_codes = {'E1', 'E2', 'E3', 'E4', 'E5', 'E6'}
    always_fatal = {'catalog-not-found', 'catalog-parse-error', 'no-catalog', 'node-malformed'}
    errors = [i for i in issues if i.code in error_codes or i.code in always_fatal]
    warnings = [i for i in issues if i.code not in error_codes and i.code not in always_fatal]
    if strict:
        errors = errors + warnings
        warnings = []
    return errors, warnings


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------


def _provenance(map_path, catalog):
    return {
        'tool_version': TOOL_VERSION,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'error_map': os.path.abspath(map_path),
        'sources': [{'path': p, 'sha256': h} for p, h in catalog.provenance_sources],
    }


def write_machine_yaml(catalog, provenance, output_path):
    """Emit machine_error_catalog.yaml.

    Uses the same schema-key constants error_catalog.cpp's
    load_machine_catalog_from_yaml expects (the C++ constants are compiled
    from the same _SCHEMA_KEY_CONSTANTS this tool imports, so the two can't
    independently drift).
    """
    doc = {}
    doc[_schema.LANGUAGES_KEY] = list(catalog.languages)
    doc[_schema.RESERVED_KEY] = {
        name: _entry_to_yaml(entry, skip_name=True)
        for name, entry in catalog.reserved.items()
    }
    doc[_schema.INSTANCES_KEY] = {
        inst_id: dict(labels) for inst_id, labels in catalog.instances.items()
    }
    doc[_schema.NODES_KEY] = {}
    for node_name, node in catalog.nodes.items():
        doc[_schema.NODES_KEY][node_name] = {
            BASE_KEY: node['base'],
            _schema.CODES_KEY: {
                local: _entry_to_yaml(entry) for local, entry in node['codes'].items()
            },
        }

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('# Auto-generated by aggregate_error_catalog.py -- do not edit manually\n')
        f.write(f"# tool_version: {provenance['tool_version']}\n")
        f.write(f"# generated_at: {provenance['generated_at']}\n")
        f.write(f"# error_map: {provenance['error_map']}\n")
        for src in provenance['sources']:
            f.write(f"# source: {src['path']} sha256:{src['sha256']}\n")
        f.write('\n')
        yaml.safe_dump({'provenance': provenance}, f, default_flow_style=False, sort_keys=False)
        f.write('\n')
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)


def _entry_to_yaml(entry, skip_name=False):
    out = {_schema.GLOBAL_KEY: entry['global']}
    if not skip_name:
        out[_schema.NAME_KEY] = entry['name']
    if entry.get('severity'):
        out[_schema.SEVERITY_KEY] = entry['severity']
    out[_schema.ACTION_KEY] = entry.get('action', 'NONE')
    if entry.get('category'):
        out[_schema.CATEGORY_KEY] = entry['category']
    if skip_name:
        # Reserved entries: the C++ loader does not read an 'instanced' key
        # here at all — it auto-derives instanced from whether any
        # description contains "{instance}" (has_instance_placeholder). An
        # explicit key would just be misread as a bogus locale (caught by
        # this tool's own W-equivalent locale cross-check). Locale text goes
        # at the top level, matching the C++ reserved-block parsing.
        out.update(entry.get('descriptions', {}))
    else:
        out[_schema.INSTANCED_KEY] = bool(entry.get('instanced', False))
        out[_schema.DESCRIPTIONS_KEY] = dict(entry.get('descriptions', {}))
    return out


def _flat_rows(catalog):
    """Flatten reserved + node entries into one list of dict rows.

    Sorted by global_code, for the JSON/MD writers' shared column layout.
    """
    rows = []
    for name, entry in catalog.reserved.items():
        rows.append({
            'global_code': entry['global'], 'node': MANAGER_NODE_NAME,
            'local_code': 0, 'name': entry['name'],
            'severity': entry.get('severity', ''), 'action': entry.get('action', 'NONE'),
            'category': entry.get('category', ''), 'instanced': entry.get('instanced', False),
            'descriptions': dict(entry.get('descriptions', {})),
        })
    for node_name, node in catalog.nodes.items():
        for local, entry in node['codes'].items():
            rows.append({
                'global_code': entry['global'], 'node': node_name, 'local_code': local,
                'name': entry['name'], 'severity': entry.get('severity', ''),
                'action': entry.get('action', 'NONE'), 'category': entry.get('category', ''),
                'instanced': entry.get('instanced', False),
                'descriptions': dict(entry.get('descriptions', {})),
            })
    rows.sort(key=lambda r: r['global_code'])
    return rows


def write_machine_json(catalog, provenance, output_path):
    """Emit machine_errors.json, the static HMI startup lookup.

    A flat, offline-readable table an HMI loads once at its own startup
    instead of calling a live lookup service.
    """
    rows = _flat_rows(catalog)
    for row in rows:
        row['en'] = row['descriptions'].get('en', '')
        for lang in catalog.languages:
            row.setdefault(lang, row['descriptions'].get(lang, ''))
    doc = {'provenance': provenance, 'languages': list(catalog.languages), 'faults': rows}
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(doc, f, indent=2, sort_keys=False)
        f.write('\n')


def write_machine_md(catalog, provenance, output_path):
    """Emit machine_errors.md — the human-readable fault sheet.

    Contains the fault table plus a reaction matrix (which PackML action each
    fault triggers) and an instances appendix. This is the one human-facing
    sheet format (no separate CSV — one source avoids keeping several
    writers in sync), carries no per-instance ordinal index
    (there's no consumer for one yet), and has no separate severity/action
    legend — divergence between the catalog's documented action and a
    node's actually-emitted action is instead flagged live by the manager
    (see PackmlManagerInterface::on_alarm_event()).
    """
    rows = _flat_rows(catalog)
    locales = ['en'] + [lang for lang in catalog.languages if lang != 'en']

    generated_line = (
        f"Generated {provenance['generated_at']} by aggregate_error_catalog.py "
        f"{provenance['tool_version']} from `{provenance['error_map']}`.")
    lines = []
    lines.append('# Machine Error Catalog')
    lines.append('')
    lines.append(generated_line)
    lines.append('')
    for src in provenance['sources']:
        lines.append(f"- `{src['path']}` (sha256:{src['sha256'][:12]}…)")
    lines.append('')
    lines.append('## Fault table')
    lines.append('')
    header = (
        ['global_code', 'node', 'local_code', 'name', 'severity', 'action', 'category',
         'instanced'] + locales)
    lines.append('| ' + ' | '.join(header) + ' |')
    lines.append('|' + '---|' * len(header))
    for row in rows:
        cells = [
            str(row['global_code']), row['node'], str(row['local_code']), row['name'],
            row['severity'], row['action'], row['category'], str(row['instanced']),
        ] + [row['descriptions'].get(loc, '') for loc in locales]
        lines.append('| ' + ' | '.join(cells) + ' |')
    lines.append('')

    lines.append('## Reaction matrix')
    lines.append('')
    lines.append('| global_code | name | action -> PackML reaction |')
    lines.append('|---|---|---|')
    for row in rows:
        lines.append(f"| {row['global_code']} | {row['name']} | {row['action']} |")
    lines.append('')

    if catalog.instances:
        lines.append('## Instances')
        lines.append('')
        header = ['id'] + locales
        lines.append('| ' + ' | '.join(header) + ' |')
        lines.append('|' + '---|' * len(header))
        for inst_id, labels in catalog.instances.items():
            cells = [inst_id] + [labels.get(loc, '') for loc in locales]
            lines.append('| ' + ' | '.join(cells) + ' |')
        lines.append('')

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))


WRITERS = {
    'yaml': ('machine_error_catalog.yaml', write_machine_yaml),
    'json': ('machine_errors.json', write_machine_json),
    'md': ('machine_errors.md', write_machine_md),
}
DEFAULT_FORMATS = ['yaml', 'json', 'md']  # no csv — one human-facing sheet format, not two


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def collect_dependencies(map_path):
    """List every file this aggregation run depends on.

    The map file itself plus every node catalog it resolves to. Shared by
    --list-inputs (configure-time DEPENDS collection) and --depfile (a
    Makefile-style .d file emitted on every run, so a catalog added to
    error_map.yaml *after* the last configure is still picked up on the next
    incremental build). Note this only sees install-space files: editing a
    node's catalog in its own source tree doesn't retrigger aggregation
    until that node package is rebuilt/reinstalled and the bringup package
    rebuilds after it — `colcon build --packages-above <node_pkg>` covers
    both.
    """
    deps = [os.path.abspath(map_path)]
    seen = set(deps)
    map_dir = os.path.dirname(os.path.abspath(map_path))
    root = load_yaml_file(map_path)
    for node_name, node_spec in (root.get(_schema.NODES_KEY, {}) or {}).items():
        node_spec = node_spec or {}
        if CATALOG_KEY not in node_spec:
            continue
        try:
            resolved = resolve_package_path(node_spec[CATALOG_KEY], map_dir)
        except Exception:
            continue  # a broken package/reference is a build-time lint error, not here
        # A resolved-but-nonexistent path (e.g. a typo'd filename inside an
        # otherwise-real package) must NOT be fed into CMake's DEPENDS/DEPFILE —
        # Make would treat a missing prerequisite with no rule to build it as a
        # hard, confusing failure ("No rule to make target ...") instead of
        # letting the real aggregation run report its own clean
        # 'catalog-not-found' diagnostic (see aggregate()'s matching isfile check).
        if os.path.isfile(resolved) and resolved not in seen:
            seen.add(resolved)
            deps.append(resolved)
    return deps


def list_inputs(map_path):
    """Print every dependency file path, one per line.

    For CMake's configure-time DEPENDS collection. Silent otherwise — the
    script dependency itself is added directly by the CMake function, not
    through this mode.
    """
    for dep in collect_dependencies(map_path):
        print(dep)


def _escape_make_path(path):
    """Escape characters significant to Make in a dependency/target path.

    Order matters: backslash first (so later-inserted escape backslashes
    aren't themselves re-escaped), then '$' (Make variable expansion), '#'
    (comment start), ':' (rule separator — breaks parsing entirely), then
    space (the original, narrower escaping this replaces).
    """
    path = path.replace('\\', '\\\\')
    path = path.replace('$', '$$')
    path = path.replace('#', r'\#')
    path = path.replace(':', r'\:')
    path = path.replace(' ', r'\ ')
    return path


def write_depfile(depfile_path, primary_output, map_path):
    """Emit a Makefile-style .d file: `<primary_output>: <dep1> <dep2> ...`.

    So an incremental build picks up dependencies discovered at run time,
    not just the ones known at the last configure.
    """
    deps = collect_dependencies(map_path)
    os.makedirs(os.path.dirname(os.path.abspath(depfile_path)), exist_ok=True)
    with open(depfile_path, 'w') as f:
        escaped = [_escape_make_path(d) for d in deps]
        f.write(f"{_escape_make_path(primary_output)}: {' '.join(escaped)}\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='Path to error_map.yaml')
    parser.add_argument('--output-dir', help='Directory for generated artifacts')
    default_formats_str = ','.join(DEFAULT_FORMATS)
    formats_help = f'Comma-separated output formats (default: {default_formats_str})'
    parser.add_argument('--formats', default=default_formats_str, help=formats_help)
    parser.add_argument('--strict', action='store_true', help='Promote warnings to errors')
    check_instances_help = 'Comma-separated instance ids that must have a label (W4)'
    parser.add_argument('--check-instances', default='', help=check_instances_help)
    list_inputs_help = 'Print dependency file paths (map + node catalogs) and exit'
    parser.add_argument('--list-inputs', action='store_true', help=list_inputs_help)
    parser.add_argument('--depfile', help='Write a Makefile-style .d dependency file here')
    args = parser.parse_args(argv)

    if args.list_inputs:
        list_inputs(args.map)
        return 0

    if not args.output_dir:
        parser.error('--output-dir is required unless --list-inputs is given')

    check_instances = [s for s in args.check_instances.split(',') if s]
    try:
        catalog, issues = aggregate(args.map, check_instances=check_instances)
    except AggregationError as e:
        sys.stderr.write(f'error: {e}\n')
        return 1
    except (AttributeError, TypeError, KeyError, IndexError) as e:
        # A wrong-shape-but-otherwise-valid YAML value (e.g. a mapping where a
        # list was expected, or vice versa) can raise one of these from deep
        # inside parsing in a way _require_scalar's leaf-level check doesn't
        # always reach first. Surface it the same clean way as an
        # AggregationError instead of an unhandled traceback — this tool's
        # whole purpose is to tolerate malformed vendor-authored input.
        sys.stderr.write(f'error: malformed error map or catalog: {e}\n')
        return 1

    errors, warnings = apply_strict(issues, args.strict)
    for w in warnings:
        sys.stderr.write(f'warning: {w}\n')
    for e in errors:
        sys.stderr.write(f'error: {e}\n')
    if errors:
        return 1

    provenance = _provenance(args.map, catalog)
    formats = [f.strip() for f in args.formats.split(',') if f.strip()]
    for fmt in formats:
        if fmt not in WRITERS:
            sys.stderr.write(f"error: unknown format '{fmt}' (known: {', '.join(WRITERS)})\n")
            return 1
        filename, writer = WRITERS[fmt]
        writer(catalog, provenance, os.path.join(args.output_dir, filename))

    if args.depfile:
        primary = (
            os.path.join(args.output_dir, WRITERS[formats[0]][0])
            if formats else args.output_dir)
        write_depfile(args.depfile, primary, args.map)

    return 0


if __name__ == '__main__':
    sys.exit(main())
