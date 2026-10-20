Copy and paste the prompt below into **Antigravity** (using Gemini Flash) with your cloned `BHumanCodeRelease` workspace open. It is structured to guide the agentic IDE through repository inspection, dependency isolation, file extraction, and standalone build verification.

```markdown
You are an expert C++ robotics software architect working inside an agentic IDE. You have full access to the workspace filesystem and terminal.

### Objective
Analyze the `BHumanCodeRelease` repository in this workspace, locate the complete high-level match strategy and behavior control engine, trace its exact file dependencies, and extract/isolate them into a self-contained directory (`standalone_strategy/`) with its own standalone `Makefile` (and `CMakeLists.txt`). The goal is to compile the strategy engine into an independent static library and test executable that has zero dependencies on the SimRobot simulator, NAO hardware drivers, or the full B-Human build system, so I can easily copy this folder into my own project later.

---

### Step-by-Step Execution Plan

#### Phase 1: Repository Analysis & Dependency Mapping
1. **Locate Strategy & Behavior Modules:**
   - Search `Src/Modules/BehaviorControl/` (specifically `StrategyBehaviorControl`, `BehaviorControl`, and `SkillBehaviorControl`) and `Src/Representations/BehaviorControl/`.
   - Identify how match strategy, team tactics (formations, Voronoi regions, role assignment), active ball play, and CABSL / CABSL 2 state machines are structured.
2. **Trace Required Representations (Inputs & Outputs):**
   - Inspect the `REQUIRES(...)`, `USES(...)`, and `PROVIDES(...)` macros in the strategy module headers.
   - List all input representations (e.g., `RobotPose`, `FieldBall`, `BallModel`, `ObstacleModel`, `GameInfo`, `RobotInfo`, `TeamData`, `FieldDimensions`) and output representations (e.g., `SkillRequest`, `MotionRequest`, `HeadMotionRequest`, `BehaviorStatus`, `StrategyStatus`).
3. **Trace Low-Level Framework Dependencies:**
   - Identify the exact headers and `.cpp` files required to compile these modules and representations:
     - Math & Geometry (`Src/Math/`, `Util/Eigen` or system Eigen3).
     - Streaming & Reflection macros (`STREAMABLE`, `ENUM` in `Src/Streaming/`).
     - CABSL behavior engine headers (`Src/Libs/cabsl/`).
     - Configuration file parsers (`InMapFile`, `OutMapFile` in `Src/Streaming/`) if needed to load `.cfg` tactic files from `Config/Scenarios/Default/Strategy/`.
4. **Deliverable (Documentation):**
   - Generate `standalone_strategy/ARCHITECTURE.md` containing:
     - A complete manifest of every extracted `.h`, `.cpp`, and `.cfg` file and its role.
     - A clear data-flow diagram showing Input Structs -> Strategy Engine -> Output Structs.

---

#### Phase 2: Decoupling & Shim Creation
To avoid pulling in the entire B-Human `Src/Framework/` threading, IPC, logging, and GUI debugging infrastructure, create lightweight shims inside `standalone_strategy/include/shims/`:
1. **Blackboard & Module Shim (`ModuleShim.h`):**
   - Replace `MODULE(...)`, `REQUIRES(...)`, and `PROVIDES(...)` macros (or provide a lightweight `StrategyBlackboard` struct) so the strategy classes inherit or reference plain C++ structs rather than B-Human's global `Blackboard` singleton.
2. **Debug & Platform Stubs (`DebugStubs.h`, `PlatformStubs.h`):**
   - Stub out B-Human debugging and drawing macros (`DECLARE_DEBUG_DRAWING`, `CIRCLE`, `LINE`, `POLYGON`, `COMPLEX_DRAWING`, `MODIFY`, `OUTPUT_TEXT`, `ANNOTATION`) as no-ops (`((void)0)`).
   - Replace `BHAssert.h` and `SystemCall::getCurrentSystemTime()` with standard C++ equivalents (`<cassert>` and `std::chrono`).

---

#### Phase 3: File Extraction & Standalone Directory Setup
Create a new self-contained directory `standalone_strategy/` at the workspace root with the following portable structure:
```text
standalone_strategy/
├── ARCHITECTURE.md
├── Makefile
├── CMakeLists.txt
├── config/                  # Copied strategy/tactic .cfg files (if applicable)
├── include/
│   ├── shims/               # Module, Blackboard, Debug, and Platform stubs
│   ├── Math/                # Extracted B-Human Math headers (Pose2f, Angle, etc.)
│   ├── Streaming/           # Extracted Streamable/Enum headers (or clean POD structs)
│   ├── cabsl/               # Extracted CABSL headers
│   ├── Representations/     # Extracted Input/Output representation headers
│   └── Strategy/            # Extracted Strategy & BehaviorControl headers
├── src/                     # Extracted Strategy, Behavior, and Math .cpp files
│   └── strategy_wrapper.cpp # Clean public C++ API wrapper class
└── main_test.cpp            # Minimal test harness verifying compilation and execution

```

*Note: Copy (do not move) the required files from `Src/` into `standalone_strategy/` and adjust `#include` paths or compiler `-I` flags so the folder is 100% self-contained and can be moved to another machine without the rest of the `BHumanCodeRelease` repo.*

---

#### Phase 4: Build System (`Makefile`) & Verification

1. **Create `standalone_strategy/Makefile`:**
* Use `g++` or `clang++` with `-std=c++20 -Wall -Wextra -O2`.
* Include necessary flags (e.g., `-Iinclude`, `-I/usr/include/eigen3`, `-Wno-gnu-zero-variadic-macro-arguments`).
* Define targets:
* `all`: Builds `libbhuman_strategy.a` (static library) and `strategy_test` (test binary).
* `lib`: Builds only `libbhuman_strategy.a`.
* `clean`: Removes build artifacts (`build/`, `*.o`, `*.a`, `strategy_test`).




2. **Create `standalone_strategy/main_test.cpp`:**
* Instantiate the decoupled strategy engine.
* Populate dummy input states (e.g., `GameInfo::state = PLAYING`, `RobotPose` at `(0, 0, 0)`, `FieldBall` at `(1500, 200)`).
* Run 5 ticks of the strategy update loop and print the resulting role assignment, `SkillRequest`, and `MotionRequest` to `stdout`.


3. **Compile and Fix Errors Iteratively:**
* Run `make -C standalone_strategy` in the terminal.
* If any missing headers, macro expansion errors, or undefined symbols occur, inspect the compiler output, resolve the dependencies or stub the unneeded calls, and re-run `make` until `libbhuman_strategy.a` and `./standalone_strategy/strategy_test` compile and execute cleanly with zero errors.
