# Cartesian Trajectory Generator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a deterministic C library that converts `current -> pick -> transit -> place` waypoints into smooth, time-parameterized Cartesian `Pose(x, y, z, yaw)` references for the STM32F407 project.

**Architecture:** Keep trajectory generation in `App/lib/trajectory` with no HAL or RTOS dependencies. Generate a rest-to-rest quintic approach and a two-segment C2 quintic Hermite transfer, then evaluate either phase from caller-supplied elapsed time. Verify behavior with a native MinGW test executable and compile the same source into the ARM firmware.

**Tech Stack:** C11, CMake 3.28, Ninja, MinGW GCC for runnable host tests, `D:/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe` for firmware, STM32F407 single-precision FPU.

## Global Constraints

- Output only Cartesian `TrajectoryPose`; inverse kinematics and servo output remain outside this module.
- Use `float` with `mm`, `deg`, and `s` units.
- Use no heap allocation, HAL symbol, FreeRTOS symbol, or task-period assumption.
- Compile firmware with `D:/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe`; use MinGW GCC only for native unit tests.
- Stop at `pick` and `place`; pass `transit` with C2 continuity whenever its safe monotone tangent is nonzero.
- Interpolate yaw over the shortest angular distance and normalize output to `[-180, 180)`.
- Enforce XYZ vector speed/acceleration and yaw speed/acceleration limits by duration scaling.
- Modify neither `Core/Src/freertos.c`, `NUEDC_2026_Control.ioc`, inverse kinematics, nor the servo driver.

---

## File Map

- Create `App/lib/trajectory/trajectory.h`: public types, enums, plan storage, and API declarations.
- Create `App/lib/trajectory/trajectory.c`: validation, yaw handling, quintic generation, constraint scaling, and evaluation.
- Create `App/lib/trajectory/README.md`: units, phase sequencing, API example, and degenerate-path behavior.
- Create `tests/trajectory/CMakeLists.txt`: native test executable independent from the ARM toolchain.
- Create `tests/trajectory/trajectory_test.c`: deterministic unit tests and numerical assertions.
- Modify `CMakeLists.txt`: compile the trajectory library into the firmware and expose its include directory.

---

### Task 1: Public API, Native Test Harness, and Approach Phase

**Files:**
- Create: `App/lib/trajectory/trajectory.h`
- Create: `App/lib/trajectory/trajectory.c`
- Create: `tests/trajectory/CMakeLists.txt`
- Create: `tests/trajectory/trajectory_test.c`

**Interfaces:**
- Consumes: `TrajectoryRequest` containing four poses and four positive motion limits.
- Produces: `Trajectory_Generate`, `Trajectory_Evaluate`, and `Trajectory_GetDuration` exactly as specified in the design document.

- [ ] **Step 1: Create the native test target and write failing API/approach tests**

Create `tests/trajectory/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(trajectory_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(trajectory_test
    trajectory_test.c
    ../../App/lib/trajectory/trajectory.c
)

target_include_directories(trajectory_test PRIVATE
    ../../App/lib/trajectory
)

target_compile_options(trajectory_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(trajectory_test PRIVATE m)

add_test(NAME trajectory_test COMMAND trajectory_test)
```

Start `trajectory_test.c` with assertion helpers and these tests:

```c
static void test_approach_endpoints_and_midpoint(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryPose pose;
    float duration;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    duration = Trajectory_GetDuration(&plan, TRAJECTORY_PHASE_APPROACH);
    ASSERT_TRUE(duration > 0.0f);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      0.0f, &pose));
    assert_pose_near(request.current, pose, 1.0e-5f);

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      duration * 0.5f, &pose));
    ASSERT_NEAR((request.current.x_mm + request.pick.x_mm) * 0.5f,
                pose.x_mm, 1.0e-4f);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                                      duration, &pose));
    assert_pose_near(request.pick, pose, 1.0e-4f);
}

static void test_invalid_request_is_rejected(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_ARGUMENT,
                  Trajectory_Generate(NULL, &plan));
    request.current.x_mm = NAN;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_ARGUMENT,
                  Trajectory_Generate(&request, &plan));
    request = valid_request();
    request.limits.max_linear_velocity_mm_s = 0.0f;
    ASSERT_EQ_INT(TRAJECTORY_RESULT_INVALID_LIMIT,
                  Trajectory_Generate(&request, &plan));
}
```

Use this baseline request in `valid_request()`:

```c
TrajectoryRequest request = {
    .current = {20.0f, 30.0f, 45.0f, 170.0f},
    .pick = {80.0f, 60.0f, 5.0f, -170.0f},
    .transit = {120.0f, 130.0f, 50.0f, -120.0f},
    .place = {160.0f, 200.0f, 5.0f, -90.0f},
    .limits = {120.0f, 300.0f, 90.0f, 180.0f}
};
```

- [ ] **Step 2: Configure and run the tests to verify RED**

Run:

```powershell
cmake -S tests/trajectory -B build/trajectory-host -G Ninja `
  -DCMAKE_C_COMPILER=D:/MinGW/mingw64/bin/gcc.exe
cmake --build build/trajectory-host
```

Expected: compilation fails because `trajectory.h` and the public API do not exist.

- [ ] **Step 3: Implement the public header and minimal approach generator**

Define in `trajectory.h`:

```c
#define TRAJECTORY_AXIS_COUNT 4U
#define TRAJECTORY_COEFFICIENT_COUNT 6U
#define TRAJECTORY_TRANSFER_SEGMENT_COUNT 2U

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float yaw_deg;
} TrajectoryPose;

typedef struct {
    float max_linear_velocity_mm_s;
    float max_linear_acceleration_mm_s2;
    float max_yaw_velocity_deg_s;
    float max_yaw_acceleration_deg_s2;
} TrajectoryLimits;

typedef struct {
    TrajectoryPose current;
    TrajectoryPose pick;
    TrajectoryPose transit;
    TrajectoryPose place;
    TrajectoryLimits limits;
} TrajectoryRequest;

typedef enum {
    TRAJECTORY_PHASE_APPROACH = 0,
    TRAJECTORY_PHASE_TRANSFER = 1
} TrajectoryPhase;

typedef enum {
    TRAJECTORY_RESULT_OK = 0,
    TRAJECTORY_RESULT_INVALID_ARGUMENT,
    TRAJECTORY_RESULT_INVALID_LIMIT,
    TRAJECTORY_RESULT_NUMERIC_ERROR
} TrajectoryResult;

typedef enum {
    TRAJECTORY_STATE_RUNNING = 0,
    TRAJECTORY_STATE_COMPLETE,
    TRAJECTORY_STATE_INVALID_ARGUMENT,
    TRAJECTORY_STATE_INVALID_PHASE
} TrajectoryState;

typedef struct {
    float coefficient[TRAJECTORY_AXIS_COUNT][TRAJECTORY_COEFFICIENT_COUNT];
    float duration_s;
} TrajectorySegment;

typedef struct {
    TrajectorySegment approach;
    TrajectorySegment transfer[TRAJECTORY_TRANSFER_SEGMENT_COUNT];
    float transfer_duration_s;
} TrajectoryPlan;
```

Implement validation with `isfinite`, shortest yaw delta with `fmodf`, the smoothstep power coefficients `[p0, 0, 0, 10d, -15d, 6d]`, duration computation using peak constants `1.875f` and `5.773503f`, Horner evaluation, endpoint clamping, and angle normalization.

- [ ] **Step 4: Run the native test to verify GREEN**

Run:

```powershell
cmake --build build/trajectory-host
ctest --test-dir build/trajectory-host --output-on-failure
```

Expected: `trajectory_test` passes with zero failed tests.

- [ ] **Step 5: Commit the independently working approach phase**

```powershell
git add App/lib/trajectory/trajectory.h App/lib/trajectory/trajectory.c `
  tests/trajectory/CMakeLists.txt tests/trajectory/trajectory_test.c
git commit -m "feat(trajectory): add Cartesian approach generator"
```

---

### Task 2: C2 Transfer Spline Through the Safety Point

**Files:**
- Modify: `App/lib/trajectory/trajectory.c`
- Modify: `tests/trajectory/trajectory_test.c`

**Interfaces:**
- Consumes: `pick`, `transit`, and `place` from the existing `TrajectoryRequest`.
- Produces: `TRAJECTORY_PHASE_TRANSFER` evaluation across two stored `TrajectorySegment` values.

- [ ] **Step 1: Add failing transfer waypoint and continuity tests**

Add tests that read the first transfer segment duration from the caller-owned plan:

```c
static void test_transfer_passes_transit_and_finishes_at_place(void)
{
    TrajectoryRequest request = valid_request();
    TrajectoryPlan plan;
    TrajectoryPose pose;
    float transit_time;

    ASSERT_EQ_INT(TRAJECTORY_RESULT_OK,
                  Trajectory_Generate(&request, &plan));
    transit_time = plan.transfer[0].duration_s;

    ASSERT_EQ_INT(TRAJECTORY_STATE_RUNNING,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      transit_time, &pose));
    assert_pose_near(request.transit, pose, 1.0e-3f);

    ASSERT_EQ_INT(TRAJECTORY_STATE_COMPLETE,
                  Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                                      plan.transfer_duration_s, &pose));
    assert_pose_near(request.place, pose, 1.0e-3f);
}
```

Add `evaluate_axis_derivative(segment, axis, local_time_s, order)` in the test and assert at `transit`:

```c
ASSERT_NEAR(left_velocity, right_velocity, 1.0e-3f);
ASSERT_NEAR(left_acceleration, right_acceleration, 1.0e-2f);
ASSERT_TRUE(vector_speed_at_transit > 1.0e-3f);
```

Check all four axes; the vector-speed assertion uses XYZ only.

- [ ] **Step 2: Run the transfer tests to verify RED**

Run:

```powershell
cmake --build build/trajectory-host
ctest --test-dir build/trajectory-host --output-on-failure
```

Expected: transfer waypoint or continuity assertions fail because the transfer coefficients are not generated.

- [ ] **Step 3: Implement quintic Hermite segments and the limited transit tangent**

Use normalized local time `u = t / T`. For each axis, convert physical boundary derivatives to polynomial coefficients:

```c
c0 = p0;
c1 = v0 * T;
c2 = 0.5f * a0 * T * T;

A = p1 - (c0 + c1 + c2);
B = v1 * T - (c1 + 2.0f * c2);
C = a1 * T * T - 2.0f * c2;

c3 = 10.0f * A - 4.0f * B + 0.5f * C;
c4 = -15.0f * A + 7.0f * B - C;
c5 = 6.0f * A - 3.0f * B + 0.5f * C;
```

For each XYZ component and unwrapped yaw, calculate adjacent physical secants `s0` and `s1`. Use zero transit velocity when `s0 * s1 <= 0`; otherwise use the harmonic mean and clamp it to three times the smaller secant magnitude:

```c
via_velocity = 2.0f * s0 * s1 / (s0 + s1);
limit = 3.0f * fminf(fabsf(s0), fabsf(s1));
via_velocity = clamp_abs(via_velocity, limit);
```

Set transit acceleration to zero on both segments. Handle a zero-length transfer segment as a constant, zero-duration segment and evaluate the remaining segment without division by zero.

- [ ] **Step 4: Run all native tests to verify GREEN**

Run:

```powershell
cmake --build build/trajectory-host
ctest --test-dir build/trajectory-host --output-on-failure
```

Expected: endpoint, safety-point, C2 continuity, and non-stop tests all pass.

- [ ] **Step 5: Commit the transfer spline**

```powershell
git add App/lib/trajectory/trajectory.c tests/trajectory/trajectory_test.c
git commit -m "feat(trajectory): add smooth transfer spline"
```

---

### Task 3: Guaranteed Constraint Scaling, Yaw Wrapping, and Edge Cases

**Files:**
- Modify: `App/lib/trajectory/trajectory.c`
- Modify: `tests/trajectory/trajectory_test.c`

**Interfaces:**
- Consumes: four motion limits already present in `TrajectoryLimits`.
- Produces: phase durations whose polynomial derivative bounds do not exceed those limits.

- [ ] **Step 1: Add failing limit, yaw, and degenerate-path tests**

Add a derivative sweep using the stored polynomial coefficients. At 1001 evenly spaced samples per nonzero segment, assert:

```c
ASSERT_TRUE(linear_speed <= request.limits.max_linear_velocity_mm_s + 1.0e-3f);
ASSERT_TRUE(linear_acceleration <= request.limits.max_linear_acceleration_mm_s2 + 1.0e-2f);
ASSERT_TRUE(fabsf(yaw_speed) <= request.limits.max_yaw_velocity_deg_s + 1.0e-3f);
ASSERT_TRUE(fabsf(yaw_acceleration) <= request.limits.max_yaw_acceleration_deg_s2 + 1.0e-2f);
```

Add shortest-yaw behavior:

```c
request.current.yaw_deg = 170.0f;
request.pick.yaw_deg = -170.0f;
Trajectory_Generate(&request, &plan);
Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                    plan.approach.duration_s * 0.5f, &pose);
ASSERT_NEAR(-180.0f, pose.yaw_deg, 1.0e-3f);
```

Add zero-distance and repeated-waypoint cases, asserting finite output and immediate completion for a fully stationary phase. Add an all-components-reversing transit case and assert a zero transit speed rather than spatial overshoot.

- [ ] **Step 2: Run the new tests to verify RED**

Run:

```powershell
cmake --build build/trajectory-host
ctest --test-dir build/trajectory-host --output-on-failure
```

Expected: at least one derivative-limit assertion fails before conservative transfer scaling is implemented.

- [ ] **Step 3: Implement conservative Bezier derivative bounds and duration scaling**

For each quintic axis, derive Bezier boundary control points from physical boundary values:

```c
b0 = p0;
b1 = p0 + v0 * T / 5.0f;
b2 = p0 + 2.0f * v0 * T / 5.0f + a0 * T * T / 20.0f;
b5 = p1;
b4 = p1 - v1 * T / 5.0f;
b3 = p1 - 2.0f * v1 * T / 5.0f + a1 * T * T / 20.0f;
```

Calculate first- and second-derivative Bezier control points:

```c
d1[i] = 5.0f * (b[i + 1] - b[i]);
d2[i] = 4.0f * (d1[i + 1] - d1[i]);
```

For XYZ, take the largest vector norm across corresponding control points; for yaw, take the largest absolute value. Convert normalized derivative bounds to physical bounds by dividing by `T` or `T*T`.

For each phase, apply one synchronized scale:

```c
scale = max4(linear_velocity_bound / linear_velocity_limit,
             sqrtf(linear_acceleration_bound / linear_acceleration_limit),
             yaw_velocity_bound / yaw_velocity_limit,
             sqrtf(yaw_acceleration_bound / yaw_acceleration_limit));
```

If `scale > 1.0f`, multiply every nonzero segment duration in that phase by `scale * 1.0001f`. Normalized polynomial coefficients stay unchanged, so geometry and C2 continuity remain unchanged while physical derivatives decrease.

- [ ] **Step 4: Run native tests and strict warnings to verify GREEN**

Run:

```powershell
cmake --build build/trajectory-host --clean-first
ctest --test-dir build/trajectory-host --output-on-failure
```

Expected: all tests pass and GCC emits no warnings under `-Wall -Wextra -Werror`.

- [ ] **Step 5: Commit constraint enforcement and edge-case handling**

```powershell
git add App/lib/trajectory/trajectory.c tests/trajectory/trajectory_test.c
git commit -m "test(trajectory): enforce motion limits and edge cases"
```

---

### Task 4: Firmware Integration, Documentation, and Final Verification

**Files:**
- Create: `App/lib/trajectory/README.md`
- Modify: `CMakeLists.txt`
- Verify: `App/lib/trajectory/trajectory.h`
- Verify: `App/lib/trajectory/trajectory.c`
- Verify: `tests/trajectory/trajectory_test.c`

**Interfaces:**
- Consumes: the completed pure C trajectory library.
- Produces: an ARM firmware build containing the library and usage documentation for `Route_planning_App` integration.

- [ ] **Step 1: Add the library to the firmware build**

Add to `target_sources`:

```cmake
App/lib/trajectory/trajectory.c
```

Add to `target_include_directories`:

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/App/lib/trajectory
```

- [ ] **Step 2: Document phase sequencing and a complete usage example**

The README must show this exact control flow:

```c
TrajectoryPlan plan;
TrajectoryPose reference;

if (Trajectory_Generate(&request, &plan) != TRAJECTORY_RESULT_OK) {
    /* Reject the request and report a planning fault. */
}

Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_APPROACH,
                    approach_elapsed_s, &reference);

/* After APPROACH completes, enable the magnet and confirm pickup. */

Trajectory_Evaluate(&plan, TRAJECTORY_PHASE_TRANSFER,
                    transfer_elapsed_s, &reference);
```

Document units, shortest-yaw behavior, caller-owned elapsed time, zero-distance completion, and the safe-stop fallback for a fully reversing transit point.

- [ ] **Step 3: Run final native and ARM verification**

Run:

```powershell
cmake --build build/trajectory-host --clean-first
ctest --test-dir build/trajectory-host --output-on-failure
cmake --preset Debug
$compilerConfig = Get-ChildItem build/Debug/CMakeFiles -Recurse `
  -Filter CMakeCCompiler.cmake | Select-Object -First 1
$compilerMatch = Select-String -LiteralPath $compilerConfig.FullName `
  -SimpleMatch 'set(CMAKE_C_COMPILER "D:/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")'
if (-not $compilerMatch) { throw 'Unexpected ARM compiler path' }
cmake --build --preset Debug --clean-first --parallel
git diff --check
rg -n "(HAL_|FreeRTOS|cmsis_os|Servo_)" App/lib/trajectory
```

Expected:

- native tests report zero failures;
- CMake cache reports the exact ARM compiler path specified above;
- `NUEDC_2026_Control.elf` links successfully;
- `git diff --check` exits zero;
- the dependency scan returns no matches from `trajectory.c` or `trajectory.h` (README example text is also expected to avoid these symbols).

- [ ] **Step 4: Review the final diff against the design scope**

Run:

```powershell
git status --short
git diff --stat
git diff -- CMakeLists.txt App/lib/trajectory tests/trajectory
```

Expected: only `CMakeLists.txt`, `App/lib/trajectory/*`, and `tests/trajectory/*` differ from the last implementation commit.

- [ ] **Step 5: Commit firmware integration and documentation**

```powershell
git add CMakeLists.txt App/lib/trajectory/README.md
git commit -m "docs(trajectory): integrate and document generator"
```

After the commit, rerun `git status --short`; expected output is empty.
