# TimedCoconut

**A Timing-Aware Extension of the Coconut Typestate Tool**


TimedCoconut extends the Coconut typestate system for C++ by adding timing-aware verification. In addition to checking whether method calls follow the permitted typestate transitions, it checks timing constraints and performs target-specific worst-case execution-time (WCET) analysis for embedded programs. WCET analysis estimates an upper bound on how long an analysed operation may take on the selected target, allowing timing requirements to be checked against the generated program rather than only against source-level assumptions.

TimedCoconut checks three main properties:

1. **Protocol correctness** — method calls must follow the transitions defined by the typestate specification. This prevents an operation from being invoked when the object is in a state from which that operation is not permitted.
2. **Temporal compliance** — implementation-level timing and analysed WCET values must satisfy the timing constraints associated with the typestate. In other words, a behaviourally valid transition must also complete within its specified timing interval.
3. **Architecture-aware validation** — WCET analysis and the final timing checks are performed for the selected target architecture. This is important because the same source-level operation can have different execution-time bounds on different processors or build configurations.


**Artefact URL:** ``

Download the artefact from the URL above. It is provided as a compressed file, extract it first. Then change into the extracted artefact directory and follow the Docker-based instructions in this README. The supplied Docker image provides the toolchains and analysis dependencies used by the evaluation, reducing the amount of host-specific setup required.

For example:

```bash
unzip <artefact-file>.zip
```

or, for a `.tar.gz` archive:

```bash
tar -xzf <artefact-file>.tar.gz
```

After extraction, change into the artefact directory: 

```bash
cd TimedCoconut
```

and continue with the **Getting Started Guide** below.


# Requirements

The supplied Docker image is the recommended way to evaluate the artefact because it provides a consistent software environment containing the required compiler toolchains, analysis tools, and experiment dependencies.

Required on the host machine:

- **Docker Desktop or Docker Engine:** https://www.docker.com/get-started/
- **Network connection:** required to download/install Docker, VS Code, or any missing host dependencies. Once the supplied Docker image has been loaded, most artefact steps can be run offline.
- **A terminal**
- **Sufficient disk space** to load and run the Docker image

Optional:

- **Visual Studio Code:** https://code.visualstudio.com/
- Another text editor may be used instead of VS Code.
- Physical ARM/AVR hardware for reproducing the hardware-dependent measurements reported in the paper.


# Contents of the Artefact

The artefact contains the TimedCoconut compiler plugin, the timed typestate library, five case studies, target-specific build files, the Bound-T integration used for WCET analysis, and the scripts used to reproduce the evaluation.

The main directories are:

- `include/`: TimedCoconut and typestate library headers.
- `TrafficLightController/`: TrafficLight case study.
- `TimedPillbox/`: Pillbox case study.
- `UAVDrone/`: Drone case study.
- `Robot_System/`: Robot case study.
- `Expr/`: complexity and verification-time experiment scripts.
- `cmake/`: target toolchain and linker configuration.
- `ARM-Simulate/`: supplied AArch64/QEMU executables and CSV data used for the execution comparison.
- `src/`: TimedCoconut source code

The supplied Docker image is:

```text
timedcoconut:1.3
```


# Target Hardware and Sensitivity of the Artefact

TimedCoconut is target-sensitive: the WCET values and the final timing-verification result depend on the architecture and build configuration used to generate the binary. Therefore, WCET values should be interpreted together with the target, clock configuration, compiler, and analysis settings under which they were produced.

The benchmark suite contains both illustrative examples and case studies adapted from existing software systems. The Pillbox, Robot, and Drone case studies are adapted from developed systems rather than being created specifically for TimedCoconut. Their application logic is preserved across the ARM and AVR variants; only target-specific implementation details, such as delay or wait routines, are changed where required by the architecture.

The artefact contains two verification targets:

1. **AVR:** ATmega2560, as used by the Arduino Mega 2560. The build uses `-mmcu=atmega2560` and `-DF_CPU=16000000UL`.
2. **ARM:** ARM7TDMI/ARM7 target configuration. The build uses `-mcpu=arm7tdmi` and the ARM7 Bound-T analysis path.

The ARM and AVR versions of each case study use the same application logic, typestate transitions, and timing requirements. Separate source variants are provided only where a platform-specific delay or wait implementation is required. A delay mechanism suitable for one architecture may not be appropriate for the ATmega2560, so the AVR variant uses the corresponding AVR-specific mechanism. These platform-level changes preserve the intended application behaviour and timed typestate specification while allowing each target to be analysed using an implementation appropriate to that architecture.


For reproducibility, use each supplied source variant with its corresponding toolchain and build configuration. Changing the MCU or CPU, clock frequency, compiler version, optimisation level, linker configuration, delay implementation, or WCET-analysis model may change the reported WCET values and, in some cases, may change whether a timing contract passes or fails. Reproduction should therefore use the supplied configuration unless the purpose of the experiment is specifically to study target sensitivity.


# Getting Started Guide

The commands in this section reproduce the basic artefact workflow: loading the supplied environment, configuring both targets, verifying the TrafficLight example, and running the evaluation scripts. The TrafficLight example is used first because its cyclic transition sequence and timing budget are easy to inspect. The complete five-case-study ARM7/AVR build matrix is provided under Claim 4.

## 1. Load the Docker Image

Run the Docker commands in this section from the host terminal. After the container has started, the README explicitly indicates when subsequent commands should be executed inside the container.

If Docker is not installed, download it from:

```text
https://www.docker.com/get-started/
```

VS Code is optional and can be downloaded from:

```text
https://code.visualstudio.com/
```

Check that Docker is installed and available from the terminal:

```bash
docker --version
```

For a compressed image on Linux, macOS, or WSL:

```bash
gzip -dc timedcoconut-1.3-amd64.tar.gz | docker load
```

For an uncompressed image:

```bash
docker load -i timedcoconut-1.3-amd64.tar
```

On Windows PowerShell, extract the `.tar.gz` file first and run:

```powershell
docker load -i .\timedcoconut-1.3-amd64.tar
```

Check that the image was loaded successfully:

```bash
docker images timedcoconut
```

The expected image entry is:

```text
timedcoconut:1.3
```

## 2. Start the Container

Linux, macOS, or WSL:

```bash
docker run --rm -it \
  --platform linux/amd64 \
  timedcoconut:1.3
```

PowerShell:

```powershell
docker run --rm -it --platform linux/amd64 timedcoconut:1.3
```

All remaining commands are run inside the container unless stated otherwise. The workspace inside the image is available under `/workspace`.

Check that the main build, WCET-analysis, complexity-analysis, and Python dependencies are available:

```bash
ls /workspace
command -v cmake
command -v avr-gcc
command -v arm-none-eabi-gcc
command -v boundt_avr
command -v boundt_arm7
command -v pmccabe
python3 --version
python3 -c "import pandas, numpy, matplotlib, seaborn; print('Python packages are ready')"
```

## 3. Configure and Build ARM7

```bash
cd /workspace
rm -rf build-arm7

cmake -S . -B build-arm7 \
  -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm7.cmake \
  -DBUILD_ARM7=ON \
  -DBUILD_AVR=OFF \
  -DBUILD_DESKTOP=OFF
```
Output:
```
-- The C compiler identification is GNU 10.3.1
-- The CXX compiler identification is GNU 10.3.1
-- The ASM compiler identification is GNU
-- Found assembler: /opt/arm10/bin/arm-none-eabi-gcc
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /opt/arm10/bin/arm-none-eabi-gcc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /opt/arm10/bin/arm-none-eabi-g++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- ARM GCC plugin dir: /opt/arm10/bin/../lib/gcc/arm-none-eabi/10.3.1/plugin
-- ARM7 foodmixer build -> /workspace/build-arm7/foodmixer_arm7.elf + .bin
-- ARM7 robot build -> /workspace/build-arm7/robot_arm7.elf + .bin
-- ARM7 trafficlight build -> /workspace/build-arm7/trafficlight_arm7.elf + .bin
-- ARM7 drone build -> /workspace/build-arm7/drone_arm7.elf + .bin
-- ARM7 pillbox build -> /workspace/build-arm7/pillbox_arm7.elf + .bin
-- Using ARM7 linker script: /workspace/cmake/ldscripts/arm7.ld copied to /workspace/build-arm7/arm7.ld
-- Build mode → DESKTOP=OFF  AVR=OFF  ARM7=ON
-- Configuring done (1.9s)
-- Generating done (0.0s)
-- Build files have been written to: /workspace/build-arm7

```
Build the TrafficLight example. The `_twopass` target runs the analysis/annotation pass and then rebuilds the annotated source for final verification:

```bash
cmake --build build-arm7 \
  --target trafficlight_arm7_twopass \
  -- -j1
```

A valid build should complete both verification passes, produce the WCET trace, and end with messages similar to the following. Minor formatting differences in the output are acceptable; the important point is that behaviour checking, WCET analysis, automatic annotation, the second verification pass, and the architecture-specific check all complete successfully:

```text
✓ All behaviour checks passed
✓ Auto timing analysis done: Bound-T → JSON completed successfully
✓ Auto annotation done: 4 WCET annotation(s)
✓ Timed checks verified with two-pass logic (PASS 1 analysis/annotation + PASS 2 rebuild)
✓ Architecture-aware validation passed: platform-specific ARM7 timing path
✓ Case study: trafficlight == device arm7tdmi
```

For the supplied TrafficLight case, the completed WCET trace should end at the 14,000 ms cycle boundary rather than reporting a cycle overrun. The trace accumulates the analysed WCET of each transition through one complete protocol cycle. A successful trace is similar to the following:

```text
[trace] --- longest completed WCET path (path=14000.00 ms, cycle=14000.00 ms, rows=4) ---
[trace] t_0=0.00 ms --RedAmber()--> t_1=2000.00 ms [state 0 -> 1]
[trace] t_1=2000.00 ms --Green()--> t_2=8000.00 ms [state 1 -> 2]
[trace] t_2=8000.00 ms --Amber()--> t_3=11000.00 ms [state 2 -> 3]
[trace] t_3=11000.00 ms --Red()--> t_4=14000.00 ms [state 3 -> 0]
[trace][OK] all completed WCET trace(s) are within cycle budget 14000.00 ms
```

The trace shows the verified transition order, the accumulated WCET after each transition, the source and destination typestates, and the final cycle-budget check. Reaching exactly 14,000 ms is valid here because the completed path does not exceed the 14,000 ms cycle budget.

## 4. Configure and Build AVR

```bash
cd /workspace
rm -rf build-avr

cmake -S . -B build-avr \
  -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-avr.cmake \
  -DBUILD_AVR=ON \
  -DBUILD_ARM7=OFF \
  -DBUILD_DESKTOP=OFF
```

Build the TrafficLight example. The `_twopass` target runs the analysis/annotation pass and then rebuilds the annotated source for final verification:

```bash
cmake --build build-avr \
  --target trafficlight_avr_twopass \
  -- -j1
```
Output:
```
-- The C compiler identification is GNU 7.3.0
-- The CXX compiler identification is GNU 7.3.0
-- The ASM compiler identification is GNU
-- Found assembler: /usr/bin/avr-gcc
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /usr/bin/avr-gcc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/avr-g++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- BUILD_AVR is ON
-- AVR GCC plugin dir: /usr/lib/gcc/avr/7.3.0/plugin
-- AVR foodmixer build -> /workspace/build-avr/foodmixer.elf + .hex
-- AVR trafficlight build -> /workspace/build-avr/trafficlight_avr.elf + .hex
-- AVR robot build -> /workspace/build-avr/robot_avr.elf + .hex
-- AVR pillbox build -> /workspace/build-avr/pillbox_avr.elf + .hex
-- AVR drone build -> /workspace/build-avr/drone_avr.elf + .hex
-- Build mode → DESKTOP=OFF  AVR=ON  ARM7=OFF
-- Configuring done (1.5s)
-- Generating done (0.0s)
-- Build files have been written to: /workspace/build-avr
```

A valid AVR build should complete the same two-pass verification workflow for the ATmega2560 target and end with messages similar to:

```text
✓ All behaviour checks passed
✓ Auto timing analysis done: Bound-T → JSON completed successfully
✓ Auto annotation done: 4 WCET annotation(s)
✓ Timed checks verified with two-pass logic (PASS 1 analysis/annotation + PASS 2 rebuild)
✓ Architecture-aware validation passed: platform-specific AVR timing path
✓ Case study: trafficlight_avr == device atmega2560
```
A successful trace is similar to the following:

```text
[trace] --- longest completed WCET path (path=14000.00 ms, cycle=14000.00 ms, rows=4) ---
[trace] t_0=0.00 ms --RedAmber()--> t_1=2000.00 ms [state 0 -> 1]
[trace] t_1=2000.00 ms --Green()--> t_2=8000.00 ms [state 1 -> 2]
[trace] t_2=8000.00 ms --Amber()--> t_3=11000.00 ms [state 2 -> 3]
[trace] t_3=11000.00 ms --Red()--> t_4=14000.00 ms [state 3 -> 0]
[trace][OK] all completed WCET trace(s) are within cycle budget 14000.00 ms
```


The build generates an AVR ELF file, which is used by the analysis workflow, and a HEX file that can be flashed to an ATmega2560-based board when physical hardware is available.


## 5. Run the Complexity Experiment

```bash
cd /workspace/Expr
./run_all_complexity.sh
```

The script should generate the following CSV files:

```text
FoodMixer_complexity.csv
TrafficLight_complexity.csv
Pillbox_complexity.csv
Robot_complexity.csv
Drone_complexity.csv
```

Each complexity CSV contains the following fields:

```text
Case,NLOC,Tokens,CyclomaticComplexity
```

## 6. Run the Verification and Compilation-Time Experiment

The experiment reported in the paper uses 100 repetitions. Running the full experiment can take considerably longer (approximately 37 minutes in the reported setup), so the script also accepts a smaller repetition count for a quicker artefact check. A short run is useful for confirming that the scripts execute correctly, whereas the 100-repetition run should be used when reproducing the full reported methodology.

For five repetitions:

```bash
cd /workspace/Expr
./run_all_compile_tests.sh 5
```

For the full 100 repetitions:

```bash
./run_all_compile_tests.sh 100
```

Individual case studies can also be measured separately. Use a repetition count greater than one so that the script can report both an average and a meaningful standard deviation:

```bash
./arm_drone_compile.sh 20
./avr_drone_compile.sh 20
./arm_robot_compile.sh 20
./avr_robot_compile.sh 20
```

A valid timing CSV contains one row per measurement, followed by the calculated average and standard deviation:

```text
Iteration,Compile Time (ms)
1,12345
2,12410
...
Average,12370.50
StdDev,80.125
```

An empty CSV indicates that the corresponding build or measurement did not complete successfully. Re-run that case before generating tables or plots; otherwise, the summary may contain missing or misleading results.

At the end of a successful complete run, the summary should report that all ten target/case-study combinations passed, for example:

```
All tests completed
Passed: 10
Failed: 0
Summary: /workspace/Expr/run_logs/summary.csv
```

## 7. Generate the Table and Plot

Generate the compile-time/complexity table and plot from the CSV files:


```bash
cd /workspace/Expr
python3 plot.py
```

This reproduces the type of comparison shown in Figure 4 of the paper. The absolute compile times may differ from the paper because the artefact workflow runs inside Docker, whereas the reported measurements were collected directly on the host environment used for the paper. The comparison should therefore be interpreted primarily in terms of the observed trends across case studies and targets rather than as an expectation of identical absolute timings. In general, it shows AVR is higher than ARM and more complex case studies need more time to analyse and compile. Example output is shown below:

```
Data loaded from CSV files:
    Scenario  ArmCompileTime  AVRCompileTime  Cyclomatic  NLOC  Tokens
   FoodMixer          8.7638          6.7834         9.0  74.0   305.0
TrafficLight          6.5700          9.6412        13.0 114.0   423.0
     Pillbox         30.3224         35.8790        51.0 447.0  1278.0
       Robot         42.5748         45.5384        88.0 655.0  2647.0
       Drone         32.3100         36.6188        36.0 500.0  2142.0
```


# Overview of the Claims

The normal builds in the **Getting Started Guide** provide the primary artefact check.
The source files can also be inspected during or after a build to examine the timed typestate declarations, generated `WCET_AT(...)` annotations, target-specific source variants, and the program structures analysed by TimedCoconut. These inspections complement the build output by showing where the timing contracts and generated WCET information appear in the source.

The artefact provides reproducible checks for the following claims:

1. Well-typed timed typestate specifications: Invalid timing intervals in a TimeGuard are rejected.

2. Behavioural verification: Method calls that do not follow the permitted typestate transitions are rejected at compile time.

3. Automated timing analysis and annotation: TimedCoconut runs Bound-T on the target ELF, converts the WCET results to JSON, and inserts WCET_AT(...) annotations into the source.

4. Two-pass architecture-aware verification: TimedCoconut performs target-specific WCET analysis and annotation in the first pass, then rebuilds the annotated source in the second pass to verify the timing contracts against the analysed WCET values.

5. Verification overhead analysis: The experiment measures verification and target-specific compilation time across the five case studies and both target configurations.

## Step-by-Step Instructions for Checking the Claims

Before starting a claim test, make sure the commands are being run inside the Docker container. If an earlier test modified a source file, restore that file first and use a clean target build. This avoids carrying generated annotations or deliberately invalid test changes into the next claim check.

ARM7:
```bash
cd /workspace
rm -rf build-arm7

cmake -S . -B build-arm7 \
  -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm7.cmake \
  -DBUILD_ARM7=ON \
  -DBUILD_AVR=OFF \
  -DBUILD_DESKTOP=OFF
```
AVR:

```bash
cd /workspace
rm -rf build-avr

cmake -S . -B build-avr \
  -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-avr.cmake \
  -DBUILD_AVR=ON \
  -DBUILD_ARM7=OFF \
  -DBUILD_DESKTOP=OFF
```

## Claim 1 — Well-Typed Timed Typestate Specifications (Sec 3)

TimedCoconut checks that timing information in the typestate is internally valid before accepting the program. This claim can be demonstrated by deliberately creating a `TimeGuard` interval whose lower duration bound is greater than its upper duration bound.

Open the ARM TrafficLight specification:

```bash
cd /workspace
nano TrafficLightController/TrafficLight-arm.h
```

Find a valid timing guard in the relevant `Timed_State<LS::Green, &TrafficLight::Amber, ...>` declaration. For example:

```cpp
TimeGuard<0, 0, 3000, 3000, Criticality::High>
```

Change it to an invalid interval in which the lower bound is greater than the upper bound and Ctr+S:

```cpp
TimeGuard<0, 0, 4000, 3000, Criticality::High>
```

Rebuild:

```bash
cmake --build build-arm7 \
  --target trafficlight_arm7_twopass \
  -- -j1
```

### Expected invalid result

The build should stop with a compile-time error such as:
```
error: static assertion failed: TimeGuard: lower(duration) must be <= upper(duration)
  143 |   static_assert(!(has_lower && has_upper) || (lower <= upper),
      |                 ~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~
```

Restore the valid specification:

``` bash
nano TrafficLightController/TrafficLight-arm.h
```
Then Change:
```cpp
TimeGuard<0, 0, 3000, 3000, Criticality::High>
```

Then rebuild:

```bash
cmake --build build-arm7 \
  --target trafficlight_arm7_twopass \
  -- -j1
```

### Expected valid result

The timing-specification error should disappear, and the normal two-pass build should complete successfully.

## Claim 2 — Behavioural Verification (Sec 4)

TimedCoconut tracks the current typestate and checks each method call against the transitions permitted from that state. A method call can therefore be rejected even when the method itself exists, if that call is not valid from the current protocol state.
for example, in AVR, first build with:

``` bash
cmake --build build-avr   --target trafficlight_avr_twopass   -- -j1
```

The normal successful builds report:

```text
✓ All behaviour checks passed
```

The TrafficLight case provides a simple visible example. Its valid cyclic sequence is `RedAmber() → Green() → Amber() → Red()`, after which the protocol returns to its initial state.

To check rejection of an invalid sequence, open the TrafficLight implementation and alter the call order so that a method is invoked from a state in which that transition is not permitted. One simple test is to comment out `TC.Amber()` or otherwise disturb the normal cyclic sequence so that a subsequent transition is attempted from the wrong typestate.

```bash
cd /workspace
nano TrafficLightController/main-avr.cpp
```

For example, alter the normal call order so that `Green()` or `Amber()` is invoked from an invalid typestate.

Rebuild:

```bash
cmake --build build-avr   --target trafficlight_avr_twopass   -- -j1
```

### Expected invalid result

TimedCoconut report a behavioural/typestate error, and the build fails. 


Restore the original call order:
```
  TC.RedAmber();// red & amber
   TC.Green(); // green
   TC.Amber();// amber
    TC.Red(); // red
```
Then rebuild:

```bash
cmake --build build-avr   --target trafficlight_avr_twopass   -- -j1
```



### Expected valid result

The behavioural error should disappear, and the build should again report:

```text
✓ All behaviour checks passed
```

The TrafficLight WCET/state trace shown in the Getting Started section should again follow the valid state sequence.

## Claim 3 — Automated Timing Analysis and Annotation (Sec 4)

TimedCoconut automatically runs target-specific Bound-T analysis, converts the resulting WCET information to JSON, inserts `WCET_AT(...)` annotations into the source, and then uses the annotated source in the second verification pass. This separates WCET extraction from the final timing-contract check: pass 1 obtains target-specific timing information, while pass 2 verifies the source with that information attached.

Use the Robot case study:

```bash
cd /workspace
cmake --build build-arm7 \
  --target robot_arm7_twopass \
  -- -j1
```

A successful build should include messages such as:

```text
✓ Auto timing analysis done: Bound-T → JSON completed successfully
✓ Auto annotation done: N WCET annotation(s)
```

The exact annotation count `N` depends on the case study because different case studies contain different numbers of analysed operations.

Inspect the annotated Robot source to confirm that the generated WCET annotations were inserted:

```bash
grep -n 'WCET_AT' /workspace/Robot_System/Robot-Arm.h | grep -v 'NaA'
```

The source should contain generated entries of the following form:

```cpp
WCET_AT("wcet_ms_exact=...; cycles=...");
void Robot::InitPID() {
    ...
}
```

This inspection makes the analysis pipeline visible: the target ELF is analysed, the WCET data is converted to a machine-readable form, and `WCET_AT(...)` annotations are generated for use by the second compile-time verification pass.

## Claim 4 — Two-Pass Architecture-Aware Verification Across the Case Studies (Sec 4 and 5)

This claim is checked by building the complete benchmark suite for both supported targets. Each case study follows the same two-pass workflow, but the generated machine code and WCET values are target-specific. A successful result therefore does not require ARM7 and AVR to produce identical WCET values; it requires each target-specific build to satisfy the same logical timed protocol under its own analysed timing values.

### ARM7 case studies

```bash
cd /workspace

cmake --build build-arm7 --target foodmixer_arm7_twopass -- -j1
cmake --build build-arm7 --target trafficlight_arm7_twopass -- -j1
cmake --build build-arm7 --target pillbox_arm7_twopass -- -j1
cmake --build build-arm7 --target robot_arm7_twopass -- -j1
cmake --build build-arm7 --target drone_arm7_twopass -- -j1
```

A successful ARM7 build should end with messages such as:

```text
✓ All behaviour checks passed
✓ Auto timing analysis done: Bound-T → JSON completed successfully
✓ Auto annotation done: N WCET annotation(s)
✓ Timed checks verified with two-pass logic (PASS 1 analysis/annotation + PASS 2 rebuild)
✓ Architecture-aware validation passed: platform-specific ARM7 timing path
```

### AVR case studies

```bash
cd /workspace

cmake --build build-avr --target foodmixer_twopass -- -j1
cmake --build build-avr --target trafficlight_avr_twopass -- -j1
cmake --build build-avr --target pillbox_avr_twopass -- -j1
cmake --build build-avr --target robot_avr_twopass -- -j1
cmake --build build-avr --target drone_avr_twopass -- -j1
```

A successful AVR build should end with messages such as:

```text
✓ All behaviour checks passed
✓ Auto timing analysis done: Bound-T → JSON completed successfully
✓ Auto annotation done: N WCET annotation(s)
✓ Timed checks verified with two-pass logic (PASS 1 analysis/annotation + PASS 2 rebuild)
✓ Architecture-aware validation passed: platform-specific AVR timing path
```

The annotation count `N` depends on the case study.

### What each case study demonstrates

| Case study | Main verification feature visible in the build |
| --- | --- |
| **FoodMixer** | A timed process sequence verified with target-specific WCET values. |
| **TrafficLight** | A cyclic timed protocol. The build prints the state/WCET trace and checks the 14,000 ms cycle budget. |
| **PillBox** | A larger stateful application with several timed operations and generated WCET annotations. |
| **Robot** | Cyclic control, PID-related computation, bounded iteration, conditional branching, and several timed stages within one control cycle. |
| **Drone** | A larger control-oriented case study with several analysed functions and target-specific WCET annotations. |


The case studies also demonstrate programming structures commonly used in embedded **super-loop** designs. A super-loop is a control structure in which the program repeatedly executes a fixed top-level loop while with true or for with large number of itration, calling sensing, computation, control, communication, or actuation routines in sequence rather than relying on a general-purpose operating-system scheduler. Within this structure, the supplied case studies include cyclic protocols, bounded loops, conditional branches, PID-related computation, and timed stages. TimedCoconut analyses these bounded control-flow structures as part of the target-specific WCET and typestate verification workflow.


## Invalid example 1 — Drone AVR implementation slowdown

This negative test keeps the Drone timing specification unchanged while  making one implementation operation slower. It therefore exercises the complete architecture-aware path: modified AVR code → new target binary → Bound-T WCET analysis → regenerated `WCET_AT(...)` annotation → pass-2 timing rejection. The purpose is to show that changing the implementation, that affect the contract, can cause a previously valid program to fail timing verification.

The valid `idlemotors()` implementation contains a 5 ms hardware-delay model and is checked by a timed state whose permitted duration is 5–10 ms.

#### Step 1 — Build the valid Drone first

```bash
cd /workspace
cmake --build build-avr --target drone_avr_twopass -- -j1
```

The normal build should succeed before the negative test is introduced. This establishes a valid baseline for comparison.

#### Step 2 — Back up the Drone AVR source

```bash
cp UAVDrone/Drone-avr.h UAVDrone/Drone-avr.h.claim4-backup
```

#### Step 3 — Increase the implementation delay, not the timing contract

Open the Drone AVR source:

```bash
nano UAVDrone/Drone-avr.h
```

Find `DroneController::idlemotors()`. The valid implementation is of the form:

```cpp
void DroneController::idlemotors()
{
    delay_5ms_avr(); 
    setmotors(IDLE_DC);
}
```

Temporarily change only:

```cpp
delay_5ms_avr(); ;
```

to:

```cpp
delay_19ms_avr(); 
```

Leave the corresponding `TimeGuard` unchanged. In particular, do not manually edit the generated `WCET_AT(...)` annotation; the two-pass build removes and regenerates WCET annotations from the analysed target binary.

#### Step 4 — Rebuild the modified Drone

```bash
cmake --build build-avr --target drone_avr_twopass -- -j1
```

#### Expected invalid result

```
cc1plus: error: WCET violates Time contract for 'idlemotors' (state 5): wcet=19.010000 ms, limit between=10.000 ms and 5.000 ms.
```



#### Step 5 — Restore the valid Drone source

```bash
mv UAVDrone/Drone-avr.h.claim4-backup UAVDrone/Drone-avr.h
cmake --build build-avr --target drone_avr_twopass -- -j1
```

The Drone build should succeed again after restoring the original 5 ms implementation, confirming that the failure was caused by the deliberate timing violation rather than by unrelated build state.

### Invalid example 2 — Robot AVR increased PID workload

This negative test increases the bounded computation performed by `Robot::ComputePID()` while leaving its timed typestate unchanged. Unlike the Drone test, which increases an explicit delay, this test increases actual control computation. It demonstrates that additional bounded work can increase the AVR WCET and can make a previously valid timing contract fail during pass 2.

The supplied valid implementation performs six PID calculations:

```cpp
void Robot::ComputePID() {
    command[0] = pid0.getOutput(actual[0], target_q[0]);
    command[1] = pid1.getOutput(actual[1], target_q[1]);
    command[2] = pid2.getOutput(actual[2], target_q[2]);
    command[3] = pid3.getOutput(actual[3], target_q[3]);
    command[4] = pid4.getOutput(actual[4], target_q[4]);
    command[5] = pid5.getOutput(actual[5], target_q[5]);
}
```

A representative valid AVR annotation is approximately 2 ms. However, reviewers should use the value generated by their own build rather than relying on a hard-coded WCET number, because WCET values are target- and configuration-dependent.

#### Step 1 — Build the valid Robot first

```bash
cd /workspace
cmake --build build-avr --target robot_avr_twopass -- -j1
```

Inspect the generated `ComputePID()` annotation:

```bash
grep -n -B1 -A10 "ComputePID" Robot_System/Robot-Avr.h
```

#### Step 2 — Back up the Robot AVR implementation

```bash
cp Robot_System/Robot-Avr.h Robot_System/Robot-Avr.cpp.claim4-backup
```

#### Step 3 — Repeat the existing six PID computations

Open the implementation:

```bash
nano Robot_System/Robot-Avr.h
```

Keep the original six assignments and add one extra bounded set of PID computations using temporary values:

```cpp
void Robot::ComputePID() {
    command[0] = pid0.getOutput(actual[0], target_q[0]);
    command[1] = pid1.getOutput(actual[1], target_q[1]);
    command[2] = pid2.getOutput(actual[2], target_q[2]);
    command[3] = pid3.getOutput(actual[3], target_q[3]);
    command[4] = pid4.getOutput(actual[4], target_q[4]);
    command[5] = pid5.getOutput(actual[5], target_q[5]);

    // Deliberate extra bounded PID workload for the negative test.
    const float extra0 = pid0.getOutput(actual[0], target_q[0]);
    const float extra1 = pid1.getOutput(actual[1], target_q[1]);
    const float extra2 = pid2.getOutput(actual[2], target_q[2]);
    const float extra3 = pid3.getOutput(actual[3], target_q[3]);
   const float extra4 = pid4.getOutput(actual[4], target_q[4]);
   const float extra5 = pid5.getOutput(actual[5], target_q[5]);
}
```

This deliberately increases the amount of bounded PID work without indexing beyond the six existing controllers. Keeping the additional work statically bounded is important because the WCET analysis must be able to determine a finite execution bound. Do not manually change the generated `WCET_AT(...)` annotation or the `TimeGuard`; the test is intended to show that TimedCoconut detects the implementation-level increase automatically.

#### Step 4 — Rebuild the modified Robot

```bash
cmake --build build-avr --target robot_avr_twopass -- -j1
```

Inspect the regenerated annotation:

```bash
grep -n -B1 -A10 "ComputePID" Robot_System/Robot-Avr.h
```

#### Expected invalid result

Bound-T should report a larger AVR WCET for `ComputePID()` because the modified implementation performs additional PID calculations. If one additional set still fits the existing timing bound, repeat the same extra six PID calls once more and rebuild. Keep the computation statically bounded and do not add new PID indices such as `pid6` unless the program actually defines corresponding controller and array entries.

Once the regenerated WCET exceeds the unchanged `ComputePID()` timing bound, pass 2 should reject the modified Robot implementation. This provides a second negative test in which the violation is caused by increased computation rather than by an explicit delay.


To restore the original later, use:
```bash
cp Robot_System/Robot-Avr.h.claim4-backup Robot_System/Robot-Avr.h
```

### Robot focus

The Robot case study is the main example of the more complex real-time programming patterns supported by TimedCoconut. Its control cycle includes sensor reading, motion computation, mode selection, PID-related computation, and actuator updates, allowing several forms of bounded control flow to be exercised within one timed protocol.

The implementation contains bounded loops over the robot joints. Because the number of iterations is statically known, these loops can be included in WCET analysis. The Robot typestate also contains conditional behaviour in which the selected control mode leads to different computation paths. TimedCoconut analyses the valid paths and checks the worst-duration path against the timing requirements, rather than assuming that every branch has the same execution cost.

The relevant declarations and implementations can be inspected directly in the `Robot_System/` directory.


The expected result is **not** identical ARM7 and AVR WCET values. The same logical Robot protocol is checked against the generated binary and WCET model for each target, so the reported timing values can differ while both builds still satisfy the same timing specification.

ARM:
```

[trace] --- longest completed WCET path (path=3.70 ms, cycle=11.00 ms, rows=10) ---
[trace] t_0=0.00 ms --InitPID()--> t_1=0.25 ms [state 0 -> 1]
[trace] t_1=0.25 ms --Read_force_data()--> t_2=1.07 ms [state 1 -> 2]
[trace] t_2=1.07 ms --Read_path_angle()--> t_3=1.89 ms [state 2 -> 3]
[trace] t_3=1.89 ms --ComputePathAngle()--> t_4=1.89 ms [state 3 -> 4]
[trace] t_4=1.89 ms --ConfigurePIDForMode()--> t_5=2.12 ms [state 4 -> 5]
[trace] t_5=2.12 ms --Compute_targets_slow()--> t_6=2.95 ms [state 7 -> 8]
[trace] t_6=2.95 ms --ComputePID()--> t_7=3.70 ms [state 8 -> 9]
[trace] t_7=3.70 ms --ApplyActuators()--> t_8=3.70 ms [state 9 -> 10]
[trace] t_8=3.70 ms --End()--> t_9=3.70 ms [state 10 -> 1]
```
AVR:

```
[trace] t_0=0.00 ms --InitPID()--> t_1=0.01 ms [state 0 -> 1]
[trace] t_1=0.01 ms --Read_force_data()--> t_2=0.81 ms [state 1 -> 2]
[trace] t_2=0.81 ms --Read_path_angle()--> t_3=1.64 ms [state 2 -> 3]
[trace] t_3=1.64 ms --ComputePathAngle()--> t_4=1.64 ms [state 3 -> 4]
[trace] t_4=1.64 ms --ConfigurePIDForMode()--> t_5=1.64 ms [state 4 -> 5]
[trace] t_5=1.64 ms --Compute_targets_slow()--> t_6=2.48 ms [state 7 -> 8]
[trace] t_6=2.48 ms --ComputePID()--> t_7=4.54 ms [state 8 -> 9]
[trace] t_7=4.54 ms --ApplyActuators()--> t_8=5.73 ms [state 9 -> 10]
[trace] t_8=5.73 ms --End()--> t_9=5.73 ms [state 10 -> 1]
```

## Claim 5 — Verification Analysis

The supplied experiment scripts measure source-level complexity and clean verification/compilation time for all five case studies on both targets. A clean build is used so that each timing measurement includes the intended verification workflow rather than reusing stale build products.

Run:

```bash
cd /workspace/Expr
./run_all_complexity.sh
./run_all_compile_tests.sh 5
python3 plot.py
```

For the full experiment reported in the paper:

```bash
./run_all_compile_tests.sh 100
```

The generated complexity files report non-comment lines of code (NLOC), token count, and cyclomatic complexity. The compile-time files report each individual measurement together with the average and standard deviation, which makes it possible to compare both typical cost and run-to-run variation.

A Docker run may produce different absolute times from those reported in the paper because compilation and verification time depend on the host machine and execution environment. The reproducible objective is therefore to run all case studies under one consistent environment and compare the verification cost across the benchmark suite. 

---


# Hardware-Dependent Results
The hardware-dependent results reported in the paper are not directly supported as reproducible claims by the standard artefact evaluation, because reproducing these measurements requires access to the corresponding physical target hardware.
The standard artefact workflow focuses on the claims that can be evaluated using the supplied Docker environment, including behavioural verification, timing analysis, WCET annotation, architecture-aware verification, and verification overhead.
The paper additionally reports execution-time and timing measurements collected on physical hardware. The ARM experiments were performed on a BeagleBone Black (BBB), while the AVR experiments were performed on an Arduino Mega 2560 based on the ATmega2560 microcontroller. Reproducing these measurements therefore requires access to the corresponding devices and is outside the standard Docker-based artefact evaluation.
For completeness, the artefact includes the generated AVR binaries and a separate AArch64/QEMU-based execution comparison. These materials allow reviewers to inspect or reproduce parts of the execution workflow without physical hardware; however, the QEMU results should not be interpreted as reproducing the physical hardware measurements reported in the paper.


## AVR

The AVR build generates verified ELF and HEX files for the ATmega2560.

A reviewer with an Arduino Mega 2560 board can use the supplied flashing commands in the `AVR-Hardware-run` directory to deploy the generated firmware and reproduce the hardware-dependent AVR measurements.


## ARM

The artefact generates and verifies ARM7 target binaries using the ARM7 toolchain and Bound-T analysis path.

Physical ARM hardware is required to reproduce the hardware execution-time measurements reported for the physical platform.

For simulating the physical ARM platform, the artefact also provides a separate AArch64/QEMU execution package for comparing the supplied benchmark executables and CSV data. This simulation section is provided for comparison of execution and should not be confused with the ARM7 target used by the TimedCoconut WCET-verification build above.

## Run All Five Supplied AArch64/QEMU Benchmarks

The `ARM-Simulate` directory contains supplied AArch64 executables for all five case studies. These commands **run the existing executables only**; they do not compile or re-verify them. QEMU is used here to execute the supplied AArch64 binaries in the container environment.

In this section, `HW.csv` refers to the simulated-hardware execution data used for this comparison; it does **not** refer to measurements collected from a physical device. This naming distinction is important when comparing these files with the separate physical-hardware results reported in the paper.

### Drone

```bash
cd /workspace/ARM-Simulate/Drone
qemu-aarch64-static -L /usr/aarch64-linux-gnu ./drone-arm
```

### FoodMixer

```bash
cd /workspace/ARM-Simulate/Foodmixer
qemu-aarch64-static -L /usr/aarch64-linux-gnu ./foodmixer-arm
```

### PillBox

```bash
cd /workspace/ARM-Simulate/PillBox
qemu-aarch64-static -L /usr/aarch64-linux-gnu ./pillbox-arm
```

### Robot

```bash
cd /workspace/ARM-Simulate/Robot
qemu-aarch64-static -L /usr/aarch64-linux-gnu ./robot-arm
```

### TrafficLight

```bash
cd /workspace/ARM-Simulate/TrafficLight
qemu-aarch64-static -L /usr/aarch64-linux-gnu ./trafficlight-arm
```

Each benchmark directory contains the corresponding CSV data used for the execution comparison:

- `Ideal.csv` — ideal/timed-typestate timing values.
- `WCET.csv` — WCET-based timing values.
- `HW.csv` — measurements from the simulated-hardware execution used for this comparison.
- `summary.csv` — summary values derived from the experiment data.


## Exit

When the evaluation is complete, exit the container:

```bash
exit
```
# Future Extensions of the Artefact

1. **Support for hybrid timing behaviour:** **Support for hybrid timing behaviour:** TimedCoconut could be extended to support hybrid timing behaviour in which some timing parameters are defined statically, while others are determined at runtime. For example, a delay, timeout, period, or response time of a process, or the outcome of a previous computation. This would allow subsequent timing decisions to reflect the actual execution behaviour of the system rather than relying only on values fixed before execution.

2. **Support for additional hardware platforms:** TimedCoconut could be extended beyond ARM and AVR to other architectures, such as RISC-V. This would enable the approach to be applied across a broader range of embedded processors.

3. **Support for more complex application scenarios:** TimedCoconut could be extended to applications containing several interacting and communication-timed components, allowing the approach to be explored in systems where multiple events and timing constraints must be coordinated.


