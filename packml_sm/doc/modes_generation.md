# PackML Mode Generation

`packml_sm` provides a CMake function — `packml_sm_generate_modes()` — that turns a
simple YAML file into named mode constants for both C++ and Python. This avoids
hard-coding raw integers and keeps C++ and Python in sync.


## Quick start

### 1. Define your modes in YAML

```yaml
# config/my_modes.yaml
modes:
  Invalid:     0
  Production:  1
  Maintenance: 2
  Manual:      3
```

The value `0` is conventionally `Invalid` (no active mode). Values must be unique
non-negative integers. The order determines nothing — constants are looked up by name.

### 2. Call the generator in CMakeLists.txt

```cmake
find_package(packml_sm REQUIRED)

packml_sm_generate_modes(
  ${PROJECT_NAME}           # CMake target to attach the include dir to
  config/my_modes.yaml      # path relative to CMAKE_CURRENT_SOURCE_DIR
  INCLUDE_PREFIX ${PROJECT_NAME}   # subdirectory prefix for the include path
)

# Install the generated header so downstream packages can use it
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/packml_modes/include/${PROJECT_NAME}/my_modes.hpp"
  DESTINATION "include/${PROJECT_NAME}/${PROJECT_NAME}"
)
```

`INCLUDE_PREFIX` sets the subdirectory inside the include root, so downstream code
writes `#include "my_package/my_modes.hpp"`. Using `${PROJECT_NAME}` keeps it
consistent with your package's own headers. If omitted the header lands at the root
and is included as `#include "my_modes.hpp"`.

### 3. Include in C++

```cpp
#include "my_package/my_modes.hpp"   // generated from config/my_modes.yaml

packml_sm::ModeType mode = packml_modes::Production;
sm->changeMode(packml_modes::Maintenance);

// Named string via the generated overload:
std::string name = packml_sm::to_string(packml_modes::Manual);  // "Manual"
```

Constants live in `namespace packml_modes`. Including the header anywhere in a program
also *registers* its modes with `packml_sm`, which is what makes these two work:

```cpp
packml_sm::to_string(packml_modes::Manual);   // "Manual" rather than "3"
packml_sm::is_known_mode(7);                  // true only if some header declared 7
```

Both are ordinary functions in `packml_sm/modes_registry.hpp`, pulled in by `common.hpp`,
so they are visible wherever `ModeType` is and do not depend on include order.

### 4. Use in Python (optional)

Pass `PYTHON_OUTPUT` to also generate a Python module:

```cmake
set(_modes_py_dir "${CMAKE_CURRENT_BINARY_DIR}/packml_modes/python")

packml_sm_generate_modes(
  ${PROJECT_NAME}
  config/my_modes.yaml
  INCLUDE_PREFIX ${PROJECT_NAME}
  PYTHON_OUTPUT "${_modes_py_dir}"
)

# Determine the ament Python install path
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(_py_install "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(
  DIRECTORY "${_modes_py_dir}/packml_modes"
  DESTINATION "${_py_install}"
  PATTERN "__pycache__" EXCLUDE
)
```

The generated `packml_modes` Python package exposes uppercase constants and helpers:

```python
import packml_modes

mode = packml_modes.PRODUCTION       # int value 1
name = packml_modes.mode_to_string(packml_modes.MAINTENANCE)  # "Maintenance"
print(packml_modes.ALL_MODES)        # {"Invalid": 0, "Production": 1, ...}
```


## What gets generated

Given `config/my_modes.yaml` with `INCLUDE_PREFIX my_package`, the generator
produces:

| Output | Path (build tree) | Include / import |
|--------|-------------------|-----------------|
| C++ header | `packml_modes/include/my_package/my_modes.hpp` | `#include "my_package/my_modes.hpp"` |
| Python package | `packml_modes/python/packml_modes/__init__.py` | `import packml_modes` |

After install (with the `install()` calls above):

| File | Installed path |
|------|----------------|
| C++ header | `include/my_package/my_package/my_modes.hpp` |
| Python package | `lib/pythonX.Y/site-packages/packml_modes/__init__.py` |


## The generated C++ header

```cpp
// Auto-generated from my_modes.yaml -- do not edit manually
#pragma once  // (actually uses a hash-based include guard)

#include <string>
#include "packml_sm/common.hpp"
#include "packml_sm/modes_registry.hpp"

namespace packml_modes {
  constexpr packml_sm::ModeType Invalid     = 0;
  constexpr packml_sm::ModeType Production  = 1;
  constexpr packml_sm::ModeType Maintenance = 2;
  constexpr packml_sm::ModeType Manual      = 3;

  namespace detail {
    inline const bool kModesRegistered_A1B2C3D4 = packml_sm::register_modes({
      {"Invalid", 0}, {"Production", 1}, {"Maintenance", 2}, {"Manual", 3},
    });
  }  // namespace detail
}  // namespace packml_modes
```

The registration is what a header contributes beyond its constants, and it is
deliberately not a `packml_sm::is_known_mode()` of the header's own. That function and
`to_string()` are shared symbols: a second generated header defining them is a
redefinition, and any *public* header that included one would impose its mode values on
every downstream consumer. Registering instead lets several vocabularies coexist, and lets
`packml_ros` validate a mode it cannot know at compile time.

Registration order across headers is unspecified, so a value declared twice under different
names keeps whichever registered last and reports the disagreement on stderr. Only the name
is affected — `is_known_mode()` answers true either way.


## Validating a mode

`packml_ros`'s `~/changeMode` refuses any value nothing declared, before it reaches the state
machine, because an unrecognised mode arrives with no state mask and silently removes every
command restriction the declared modes impose. It asks, in order:

1. the modes a deployment's `modes_config_file` declares, when it declares any — the narrower
   and more specific statement, since a program may link a vocabulary far wider than the
   machine in front of it is commissioned for;
2. otherwise `packml_sm::is_known_mode()`, the vocabulary of the generated headers linked in.

So a deployment can declare modes in YAML alone, with no code generation at all:

```yaml
# The spec's CLEAN and EMPTY OUT, which packml_sm's bundled set does not carry.
modes:
  Invalid: 0
  Production: 1
  Clean: 5
  EmptyOut: 7
```


## The generated Python module

```python
# Auto-generated from my_modes.yaml -- do not edit manually
INVALID:     int = 0
PRODUCTION:  int = 1
MAINTENANCE: int = 2
MANUAL:      int = 3

ALL_MODES: dict[str, int] = {"Invalid": 0, "Production": 1, ...}

def mode_to_string(mode: int) -> str: ...
```

Note: Python constants are **UPPER_CASE**; their string names (in `ALL_MODES` and
`mode_to_string`) use the original casing from the YAML.


## Reference: packml_sm default modes

`packml_sm` ships `modes/default_modes.yaml` with a minimal set and generates its own
header at install time:

```yaml
modes:
  Invalid:     0
  Production:  1
  Maintenance: 2
  Manual:      3
```

Include it as:
```cpp
#include "packml_sm/default_modes.hpp"
```

`packml_sm`'s library registers this set itself, so a program that links it and declares
nothing of its own still has those four modes to validate against.

Projects that define their own modes should **not** include this header — use their own
generated header instead. The two declare the same names in `namespace packml_modes`, so
including both in one translation unit is a redefinition. No `packml_ros` header includes
it, so this only happens if a project reaches for it deliberately.
