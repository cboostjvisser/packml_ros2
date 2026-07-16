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
        category: electrical     # free-form, documentation only
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
