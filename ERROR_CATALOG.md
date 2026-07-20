# Error catalog

`NodeHealth.error_code` is just an integer — on its own it means nothing to an
operator, an HMI, or a support engineer. The error catalog turns that integer
into a symbolic name, a human-readable (optionally multi-language)
description, a documented severity/category, and — once combined with every
other Equipment Module on the machine — a single globally-unique fault number
safe to print on an alarm screen or a fault sheet. It's an optional layer on
top of the health/heartbeat mechanism (see the main [README](README.md)): you
can raise faults with just `error_code` and never touch any of this, and
everything still works.

There are two authoring steps, done by two different people in a real
deployment: the node vendor documents their own codes, and the machine
integrator combines every node's codes into one catalog for the whole
machine.

**1. Vendor: document your node's codes.** Create `config/error_catalog.yaml`
in your node's package:

    node_name: my_motor_driver   # informational
    error_codes:                 # name -> integer, used to generate C++ constants
      OVERCURRENT: 301
      OVER_TEMPERATURE: 302
    descriptions:                # not compiled in — editing text needs re-aggregating
      OVERCURRENT:
        action: ABORT            # NONE | WARN | HOLD | SUSPEND | ABORT
        severity: CRITICAL       # CRITICAL | ERROR | WARNING | INFO
        category: electrical     # electrical | mechanical | thermal | sensor |
                                  # pneumatic | hydraulic | software |
                                  # communication | safety | process |
                                  # calibration | power | other
        en: "Motor overcurrent detected"
        nl: "Motor overstroom gedetecteerd"
      OVER_TEMPERATURE:
        action: HOLD
        severity: ERROR
        en: "Motor over-temperature"

Install it so other packages can find it:

    // CMakeLists.txt
    install(FILES config/error_catalog.yaml DESTINATION share/${PROJECT_NAME}/config)

Generate strictly-typed C++ constants from the `error_codes:` section (mirrors
`packml_sm_generate_modes`):

    // CMakeLists.txt
    find_package(packml_ros REQUIRED)
    packml_ros_generate_error_codes(${PROJECT_NAME} config/error_catalog.yaml
      NAMESPACE MyMotorErrors
      INCLUDE_PREFIX my_motor_driver)

    // your_code.cpp
    #include "my_motor_driver/error_catalog.hpp"   // generated
    h.error_code = MyMotorErrors::OVERCURRENT;

A fault that applies to one of several identical physical locations (e.g. an
E-stop wired to more than one place) can also set
`h.instance_id = "cell_north"` alongside `error_code` — see `instances:` below.

**2. Integrator: combine every node's codes for the whole machine.** In your
bringup package, create `config/error_map.yaml` listing every required node
and where its catalog lives:

    languages: [en, nl]
    reserved:
      heartbeat_timeout: { global: 1, action: ABORT, en: "Heartbeat lost" }
    nodes:
      my_motor_driver:
        catalog: package://my_motor_pkg/config/error_catalog.yaml
        base: 1000              # local code 301 becomes global 1301
      my_gripper_driver:
        catalog: package://my_gripper_pkg/config/error_catalog.yaml
        base: 2000
    instances:
      cell_north: { en: "North Cell", nl: "Noord Cel" }

`base:` must give every node a non-overlapping block of global numbers (1-99
is reserved for manager-synthesized faults like `heartbeat_timeout`); an
optional `remap:` handles a vendor code that doesn't fit its block, and an
optional `overrides:` (keyed by the vendor's own local code) lets you
substitute your own recovery-instruction text for a specific fault without
forking the vendor's file.

Each node also gets a `window:` (default 1000): its local codes must fall in
`[1, window)`, and `base + window` bounds the node's global block. **Size the
window to the vendor's whole code space, not to the codes cataloged so far.**
Hardware suppliers commonly use their controller's native fault numbers
(often four or five digits — e.g. a controller whose alarms span 0000-9999
needs `window: 10000`), and a catalog that documents only a handful of them
today will grow. A code outside the window fails the build
(`local-code-out-of-range`) rather than aggregating silently, so nothing
breaks unnoticed — but widening a window later, or adding a `remap:` per
wild code, is avoidable churn if the window matches the supplier's numbering
range from the start. Only when a vendor's native range is impractically
huge (say, eight-digit codes) does per-code `remap:` become the better tool.

Aggregate it at build time:

    // CMakeLists.txt
    find_package(packml_ros REQUIRED)
    packml_ros_aggregate_error_catalog(${PROJECT_NAME}_error_catalog
      config/error_map.yaml
      OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/error_catalog)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/error_catalog/
      DESTINATION share/${PROJECT_NAME}/config)

This runs on every build, not just when you remember to invoke it — a
misconfiguration (two nodes' codes landing on the same global number, a
`base`/`window` that's too small, a malformed catalog) fails the build with a
clear message instead of silently shipping a broken catalog. It produces
`machine_error_catalog.yaml` (loaded by the manager), `machine_errors.json`
(a static, offline fault lookup an HMI can read at its own startup), and
`machine_errors.md` (a human-readable fault sheet with a reaction matrix).

**3. Point the manager at the generated file** — normally from your launch
file, using the installed share path:

    # your_bringup.launch.py
    parameters=[{
        'error_catalog_file': os.path.join(
            get_package_share_directory('my_bringup_pkg'),
            'config', 'machine_error_catalog.yaml'),
        'language': 'en',
    }]

Every `Alarm` on `packml_alarms` then carries `global_code` (0 if no catalog
entry matched), `instance_id`, and a `message` combining the catalog's
description with the node's own free-text detail. A missing or unloadable
catalog degrades gracefully — alarms just stay unenriched, machine behavior
is never affected.

**Try it end-to-end:** `packml_health_demo.launch.py` (see the main
[README](README.md)) ships a complete, working example of both steps —
`packml_ros/examples/config/error_catalog.yaml` (the vendor side) and
`packml_ros2/config/error_map.yaml` (the integrator side).

## Pure-Python (ament_python) packages

Both build hooks above are CMake functions, but a Python-only node or bringup
package (build_type `ament_python`) has no CMakeLists.txt to call them from.
The same two scripts can be invoked from `setup.py` instead.

**Vendor side** — generate a Python constants module instead of a C++ header,
and install the catalog alongside it:

    # setup.py (before the setup() call)
    import glob, os, subprocess, sys
    from ament_index_python.packages import get_package_share_directory

    # realpath, not abspath: under `colcon build --symlink-install` this
    # setup.py runs as a symlink from the build dir — paths must resolve
    # back to the real source tree.
    _here = os.path.dirname(os.path.realpath(__file__))
    subprocess.check_call([
        sys.executable,
        os.path.join(get_package_share_directory('packml_ros'),
                     'cmake', 'generate_error_codes_header.py'),
        '--python-only',
        os.path.join(_here, 'config', 'error_catalog.yaml'),
        os.path.join(_here, 'my_node_pkg'),   # your package's module directory
        'error_codes',                        # generated submodule name
    ])

    # and in setup()'s data_files:
    ('share/' + package_name + '/config', glob.glob('config/*.yaml')),

    # your_node.py
    from my_node_pkg import error_codes
    msg.error_code = error_codes.OVERCURRENT

This writes `my_node_pkg/error_codes/__init__.py` into the source tree (add it
to `.gitignore` — it's a generated artifact, like the C++ header) rather than
into a build directory, because `colcon build --symlink-install` runs
`setup.py develop`, which has no build step that out-of-tree output could
survive. Declare `<depend>packml_ros</depend>` and
`<depend>ament_index_python</depend>` in package.xml so the generator script
is installed before your package builds.

**Integrator side** — run the aggregation from the bringup package's setup.py
the same way:

    subprocess.check_call([
        sys.executable,
        os.path.join(get_package_share_directory('packml_ros'),
                     'cmake', 'aggregate_error_catalog.py'),
        '--map', os.path.join(_here, 'config', 'error_map.yaml'),
        '--output-dir', os.path.join(_here, 'config', 'generated'),
    ])

    # data_files:
    ('share/' + package_name + '/config', glob.glob('config/generated/*')),

A lint failure (duplicate global, broken catalog reference, …) exits non-zero,
which fails `setup.py` — and with it `colcon build` — the same guarantee the
CMake hook gives. Note the trade-off versus CMake: `setup.py` only re-runs
when colcon rebuilds the package, so after editing a *referenced* vendor
catalog, rebuild the bringup package explicitly
(`colcon build --packages-select my_bringup_pkg`) — there is no `DEPFILE`
mechanism to detect that automatically.

## Vendor packages in another workspace

Nothing above requires the vendor package and the bringup package to live in
the same workspace or repository. `package://` URIs resolve through the ament
index, so a bringup package can aggregate a catalog from any *installed*
package — including one from an underlay workspace, built and sourced
separately:

    source ~/packml_ws/install/setup.bash      # provides packml_ros
    cd ~/robot_ws                              # provides the vendor node + bringup
    colcon build

The aggregation records which file it read (path + content hash) in the
generated catalog's `provenance:` block, so a stale-underlay mixup is
diagnosable after the fact. If the vendor package isn't installed (or isn't
sourced), the build fails with `catalog-not-found` rather than silently
shipping an incomplete catalog.
