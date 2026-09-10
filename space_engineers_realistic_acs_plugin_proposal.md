# Space Engineers Realistic Attitude Control System (ACS) Plugin Proposal

## Project Summary

Build a **Space Engineers plugin/mod integration** using the Pulsar ecosystem where appropriate that replaces the game's arcade-style player flight-control behavior with a more physically grounded **Attitude Control System (ACS)**.

The first Phase 0 decision gate is physics authority. A client-only implementation is acceptable for single-player and local-host experimentation only. Multiplayer support requires a verified authoritative server-side path for torque application and thrust ownership; if that cannot be established, the project must pivot to a compatible server-mod architecture before allocator work begins.

The core idea is to preserve Space Engineers' existing thruster implementation for thrust magnitude, fuel/power consumption, atmospheric effectiveness, effects, damage, and normal grid physics, while adding the missing rotational consequences of off-center thrust. Native gyros remain independent torque-producing stabilizers unless a future, explicitly selected ACS control mode changes that policy.

The plugin should initially apply only while a **human player is actively controlling a grid**. Autonomous NPC and AI-controlled grids should continue using vanilla Space Engineers behavior so that existing encounter ships, drones, and AI remain compatible.

Long-term, ACS-aware NPC ships may be supported, but that is explicitly outside the first implementation target.

---

# Motivation

Space Engineers treats gyroscopes as generic torque-producing attitude-control devices. Real spacecraft use several distinct attitude actuators, including reaction wheels, control moment gyros, thrusters, and—in atmospheric flight—aerodynamic surfaces. This proposal adds the missing thruster-generated torque while retaining gyros as independent actuators by default.

SE also simplifies thruster behavior. Thrusters create linear force without realistically applying the rotational moment associated with their actual position relative to the grid's center of mass.

This has several consequences:

- Thruster placement has little effect on handling.
- Asymmetric propulsion layouts do not naturally induce rotation.
- Ships do not require deliberate reaction-control geometry.
- Angular momentum is largely hidden from the player by vanilla gyro behavior.
- Releasing rotational input tends to feel like commanding orientation rather than commanding torque.
- A damaged or badly designed ship does not develop the control-authority limitations that would exist in a real rigid-body system.

The proposed plugin aims to correct these behaviors while retaining as much native Space Engineers functionality as possible.

---

# Primary Goal

Create a realistic player flight-control layer in which:

1. Thruster placement matters.
2. Off-center thrust creates torque.
3. Ships retain angular velocity unless counter-torque is applied.
4. The ACS determines individual thruster outputs from requested translation and rotation.
5. Native gyros remain available as independent stabilizers unless a future selected mode explicitly changes that policy.
6. Human-controlled grids use the ACS.
7. Autonomous NPC/AI-controlled grids remain vanilla by default.
8. ACS can be toggled on/off in game without manipulating files or restarting Space Engineers.

---

# High-Level Physics Model

For each thruster:

```text
Linear force:
F_i = T_i * d_i
```

Where:

- `T_i` = current commanded/effective thrust
- `d_i` = thruster force direction

The rotational contribution is:

```text
Torque:
tau_i = r_i x F_i
```

Where:

- `r_i` = vector from grid center of mass to the thruster's effective force position
- `F_i` = thruster force
- `x` = cross product

Each thruster is a bounded, single-command actuator that contributes to a six-component wrench vector:

```text
[ Fx Fy Fz Tx Ty Tz ]
```

The ACS then allocates thrust across all available thrusters.

---

# Hybrid Physics Approach

The preferred architecture is **not** to replace Space Engineers' thruster physics.

Instead:

## Space Engineers remains responsible for

- Thruster block behavior
- Maximum thrust
- Current effective thrust
- Atmospheric effectiveness
- Hydrogen consumption
- Power consumption
- Visual effects
- Damage/state
- Grid linear movement
- Havok physics
- Collisions
- Grid mass
- Center-of-mass state
- Native rigid-body integration

## The ACS plugin is responsible for

- Pilot input interpretation
- Thruster geometry analysis
- Thruster allocation
- Individual thrust command levels
- Calculating rotational moments from thruster placement
- Applying the missing torque contribution
- Suppressing vanilla gyro torque while ACS is active
- Attitude/rotation stabilization modes
- Runtime activation policy

## Force Source and Ownership Rules

The implementation must have one unambiguous source of truth for thrust and torque in each phase.

### Phase 2: torque-correction ownership

- Space Engineers owns all thruster commands and native linear force.
- The plugin derives `F_i` from the exact force actually applied by the native thruster during the same simulation step—not from maximum thrust, effective maximum thrust, a requested override, or an independently estimated value.
- The plugin adds only the corresponding missing `r_i x F_i` torque.

### Phase 4 onward: allocator ownership

- The ACS owns the individual thrust commands for an active ACS grid.
- Space Engineers continues to calculate the actual force from those commands, including fuel, power, atmospheric, damage, and other native limits.
- The ACS adds only torque omitted by native grid physics. Native gyro behavior remains unchanged unless an optional future mode explicitly owns it.
- If native availability reduces a commanded thrust, the ACS must use the resulting actual applied force for torque correction and feedback.

### ACS-off cleanup

- Restore only fields, overrides, or suppression state changed by the plugin.
- Clear plugin-owned thrust overrides before returning control to vanilla systems.
- Stop torque injection on the same transition tick, subject to the verified update-order semantics.

Conceptually:

```text
Pilot input
     |
     v
Desired wrench
[Fx Fy Fz Tx Ty Tz]
     |
     v
ACS allocator
     |
     v
Individual thruster outputs
     |
     +------------------------+
     |                        |
     v                        v
Space Engineers          ACS plugin
linear thrust            r x F torque
     |                        |
     +-----------+------------+
                 |
                 v
             Grid physics
                 |
                 v
               Havok
```

This preserves native SE behavior while restoring the rotational effect of force applied away from the center of mass.

---

# Control Model

The ACS should distinguish between **force/torque input** and **hold/stabilization behavior**.

Possible operating modes:

## Assisted / Attitude Hold

Normal player flight mode.

- Translational input requests acceleration.
- Rotational input requests angular motion.
- Releasing rotational input commands the ACS to remove angular velocity.
- ACS prioritizes maintaining commanded attitude behavior.

## Raw / Newtonian Mode

Temporary or selectable mode.

- Player input directly requests force/torque.
- Releasing controls commands zero additional force/torque.
- Existing linear and angular velocity persist.
- No automatic cancellation unless separately commanded.

## Velocity Hold

Optional later mode.

- ACS attempts to hold or return to a requested translational velocity.

## Precision Mode

Optional later mode.

- Lower control authority and/or stricter attitude stabilization.

---

# Modifier-Key Controls

The plugin should detect Shift/Ctrl modifier input and allow momentary control-mode changes.

Initial proposed behavior:

```text
Normal input
-> assisted ACS behavior

Shift + translation
-> raw/Newtonian translation behavior

Ctrl + rotation
-> direct/raw rotational torque behavior
```

Exact mappings should remain configurable.

Do not hard-code WASD assumptions where possible. Prefer Space Engineers' configured control bindings and query their current state.

---

# Thruster Allocation

The allocator receives a desired six-axis wrench:

```text
b = [Fx Fy Fz Tx Ty Tz]
```

Each thruster contributes one column to the actuator matrix:

```text
A =
[
 Fx1 Fx2 ... Fxn
 Fy1 Fy2 ... Fyn
 Fz1 Fz2 ... Fzn
 Tx1 Tx2 ... Txn
 Ty1 Ty2 ... Tyn
 Tz1 Tz2 ... Tzn
]
```

The solver determines a thrust vector:

```text
x = [t1 t2 ... tn]
```

subject to:

```text
0 <= ti <= 1
```

The system should solve approximately:

```text
A x ~= b
```

with priority handling for impossible requests.

A practical allocator does not need to be academically perfect. It needs to:

- Be stable
- Be fast
- Respect actuator limits
- Produce plausible control behavior
- Degrade gracefully when control authority is insufficient

---

# Priority Handling

The system should support prioritization when requested motion is not physically achievable.

Initial recommendation for assisted mode:

```text
1. Prevent uncontrolled rotation
2. Satisfy requested rotational control
3. Satisfy requested translation
4. Maximize achievable acceleration
```

Example:

A badly balanced ship may theoretically produce 100% forward thrust, but doing so may create more pitch or yaw torque than remaining thrusters can counter.

The allocator should therefore be allowed to restrict some thrusters below maximum.

This is desirable behavior, not a failure.

Example:

```text
Requested:
Forward = 100%
Yaw torque = 0
Pitch torque = 0
Roll torque = 0

Achievable:
Forward = 67%
Yaw torque = ~0
Pitch torque = ~0
Roll torque = ~0
```

---

# Control Authority

The plugin should eventually calculate and expose available control authority.

Example conceptual status:

```text
ACS STATUS

Translation
Forward    100%
Reverse     82%
Left        71%
Right       71%
Up          94%
Down        88%

Rotation
Pitch       76%
Yaw         91%
Roll        38%

Current limiting axis:
ROLL

Max forward thrust while holding attitude:
67%
```

This is not required for the initial feasibility spike, but the internal architecture should make it possible later.

Control authority should naturally change when:

- Thrusters are damaged
- Thrusters are destroyed
- Fuel/power availability changes
- Grid mass changes
- Center of mass moves
- Subgrids attach/detach
- Cargo shifts
- Effective thrust changes

---

# Activation Policy

Activation should be based primarily on **active grid control**, not static ownership.

## Initial V1 policy

```text
Human actively controlling grid
-> ACS enabled

Human remote-controlling grid
-> ACS enabled

AI/autopilot/NPC controller active
-> Vanilla behavior

No active human controller
-> Vanilla behavior
```

Phase 0 must turn these categories into verified, testable predicates and document the selected behavior for:

- cockpit versus passenger-seat occupancy
- remote-control blocks, including camera/spectator transitions
- turret or other non-flight control
- multiple players interacting with the same grid
- ownership/faction changes while control is active
- controller loss, grid split/merge, and remote-link loss

Until each case is verified against the current game version, it must default safely to vanilla behavior.

This avoids breaking existing NPC ships.

A hijacked NPC ship becomes realistic immediately when a player takes control.

This is intentional.

An NPC ship designed around vanilla assumptions may behave poorly under ACS:

- asymmetric thrust may cause roll/pitch/yaw
- control authority may be weak
- maximum translation may be reduced
- some maneuvers may not be achievable

This is considered interesting and physically consistent behavior.

---

# Long-Term NPC Goal

Eventually, some NPC ships may be deliberately built for ACS operation.

The architecture should therefore avoid hard-coding ACS as inherently player-only.

Recommended internal control classification:

```text
Vanilla
ACSPlayer
ACSAutonomous
```

For V1:

```text
ACSAutonomous = unused / disabled
```

Later, ACS-aware NPC blueprints and AI could use the same actuator allocator.

---

# Runtime Enable / Disable

The plugin should remain loaded but allow ACS behavior to be disabled in game.

A master runtime toggle should be available.

When ACS is OFF:

- Do not intercept pilot input
- Do not command individual thrusters
- Do not add rotational torque
- Do not suppress vanilla gyro torque
- Clear any thrust overrides owned by the plugin
- Restore vanilla behavior cleanly

The cleanup sequence must follow the ownership rules above and must be validated across controller changes as well as manual toggles.

When ACS is ON:

- Enable realistic player control for applicable grids

Target controls:

```text
Global Master Enable
Hotkey toggle
Pulsar settings/configuration
Optional future per-grid override
```

A hotkey is strongly recommended for development because it enables immediate A/B comparison between vanilla and ACS flight behavior.

---

# Per-Grid Behavior

Future support should allow:

```text
Global plugin enabled

Grid A -> ACS
Grid B -> Vanilla
Grid C -> ACS
```

Possible grid-level options:

```text
Auto
Force ACS
Force Vanilla
```

`Auto` should follow the controller policy.

This is not required for the first prototype but should be considered in the design.

---

# Gyroscope Handling

Gyros are physical torque-producing actuators. They are not an error source merely because ACS adds the missing thrust moment.

Default behavior:

```text
ACS active
-> native thrusters provide linear force
-> ACS adds the corresponding r x F torque
-> vanilla gyros retain their normal stabilization/control behavior
```

This permits a ship with active gyros to counter thrust-induced rotation, while a ship with gyros disabled can rotate freely from its thrust moment. Any future gyro-suppression or gyro-reinterpretation mode must be optional, explicitly selected, and tested separately.

## Persistent Rotational Inertia

Strict Newtonian behavior requires angular velocity to persist until another torque acts. The torque-correction phase must not fake this by injecting compensating torque.

Space Engineers exhibited residual angular damping even with gyros and inertial dampeners disabled during local tests. A later, independent inertia phase should identify and control only that native angular-damping path. It must restore original damping immediately when inertia mode is disabled and must account for collisions, landing gear, connectors, mechanical constraints, and multiplayer authority. Gyro torque remains a legitimate force that may change angular momentum.

---

# Development Environment

Primary development target:

```text
VS Code
+ Codex
+ C# tooling
+ Git
+ Pulsar client plugin template
+ searchable/decompiled Space Engineers code
```

Visual Studio or Rider may be retained as an optional debugging tool for attaching to a running Space Engineers process.

Codex should use actual game/decompiled source references wherever possible and must not invent internal Keen APIs.

---

# Critical Technical Investigation Areas

Before implementing the full ACS, inspect the current Space Engineers game code and Pulsar-accessible internals.

Identify:

## Player Input

- Active ship controller
- Human control detection
- Movement input
- Rotation input
- Configured key bindings
- Modifier keys
- Remote-control state

## Thruster System

Investigate:

- `MyGridThrustSystem`
- `MyThrust`
- How vanilla thrust requests propagate
- How individual thruster output is calculated
- Current thrust
- Effective thrust
- Maximum effective thrust
- Thruster world/local position
- Force direction
- Whether individual thrust levels can be safely controlled every tick
- Whether vanilla control must be bypassed or suppressed

## Gyroscope System

Investigate:

- `MyGyroSystem`
- Gyro torque command path
- `ControlTorque`
- Clean suppression/interception point
- Side effects of disabling gyro contribution

## Grid Physics

Investigate:

- `MyCubeGrid`
- `MyGridPhysics`
- Center of mass
- Linear velocity
- Angular velocity
- Mass/inertia data
- Safe supported methods for adding torque or angular impulse
- Simulation-tick ordering

## Grid Changes

Determine available events/hooks for:

- Thruster add/remove
- Block damage/destruction
- Grid split
- Grid merge
- Connector attach/detach
- Mechanical subgrid changes
- Center-of-mass changes

Avoid rebuilding the complete actuator model every simulation tick if event-driven or cached updates are possible.

---

# Feasibility Spike

Do **not** begin with the complete ACS.

The first implementation should be a technical feasibility experiment.

## Spike Goal

Prove that a Pulsar plugin can:

1. Identify a human-controlled grid.
2. Enumerate its thrusters.
3. Read each thruster's:
   - position
   - orientation
   - current/effective thrust
4. Determine grid center of mass.
5. Calculate each thruster's expected torque:
   ```text
   tau = r x F
   ```
6. Apply the missing torque to the grid.
7. Disable the behavior at runtime.
8. Restore vanilla behavior cleanly.

The spike must log the exact force value used, force and torque coordinate frames, simulation update phase, and—where multiplayer is in scope—whether the observed result remains authoritative after replication.

## Test Craft

Build a deliberately asymmetric test grid:

```text
        [Thruster]
             ^
             |
             |
             |
          [COM]
```

Expected behavior:

```text
ACS OFF:
thruster produces vanilla translation only

ACS ON:
same thruster produces:
- vanilla translation
- plugin-generated rotational moment
```

If this works reliably, the fundamental architecture is validated.

---

# Phase Plan

## Phase 0 - Research / Reconnaissance

No gameplay changes.

Deliver a technical report documenting:

- Space Engineers build number, Pulsar version, and the exact assemblies/decompiled source examined
- Exact classes
- Exact fully qualified member signatures and access level
- Candidate hook points
- Relevant update order
- Thruster state access
- Torque application mechanism
- Gyro interception mechanism
- Player-controller detection
- Risks and unknowns

Do not proceed based on guessed API names.

For every claimed member, record a source location and verification status. Distinguish directly verified facts from hypotheses that require an in-game experiment.

### Mandatory Decision Gate: Physics Authority

Before Phase 1, establish whether the chosen integration can apply grid torque authoritatively in each intended environment.

```text
Single-player / local host verified
-> Phase 1 may proceed

Dedicated or remote multiplayer server authority verified
-> multiplayer-capable path may proceed

Client-only local mutation or replication correction
-> limit the spike to local testing and pivot to a server-compatible mod architecture before Phase 4
```

---

## Phase 1 - Instrumentation Plugin

Create a Pulsar plugin that reports:

```text
Grid:
- mass
- COM
- linear velocity
- angular velocity
- controller type
- human-controlled status

Thruster:
- block ID
- position
- direction
- maximum thrust
- effective thrust
- current thrust
- lever arm
- calculated torque
```

Output may initially use logs/debug UI.

No flight behavior modification except optional diagnostic toggles.

---

## Phase 2 - Torque Correction

Allow vanilla thrusters to operate normally.

For each active thruster:

```text
read the exact native force applied in the relevant simulation step
calculate r x F from that actual force
apply corresponding grid torque
```

Requirements:

- Runtime ON/OFF toggle
- Player-controlled grids only
- NPC/AI grids remain vanilla
- No gyro suppression yet unless necessary for testing

Success criterion:

The asymmetric test craft produces the predicted torque sign and measured initial angular acceleration within the pre-defined tolerance, without changing the native linear-force result or leaving torque after disable.

---

## Phase 3 - Gyro Interaction and Emergency Protection

Keep vanilla gyros active by default and verify that they can stabilize the physical thrust moment without instability. The default physical-torque path targets 100% of the calculated moment; a high emergency angular-speed cutoff may suppress only torque that would increase an already unsafe spin, while still allowing counter-torque.

Success criterion:

Gyro-enabled grids stabilize predictably, gyro-disabled grids rotate from the calculated thrust moment, and the emergency cutoff prevents runaway without altering normal flight.

## Phase 3A - Persistent-Inertia Investigation

Future, separate work. Identify the native source of residual angular damping and evaluate a reversible inertia mode. Do not compensate damping with artificial ACS torque. Validate gyro-off drift, gyro-on stabilization, collisions, constrained grids, and server authority before enabling this mode by default.

---

## Phase 4 - Thruster Ownership / Manual Allocation

Take control of individual thruster commands.

Apply the Phase 4 ownership rules: ACS commands individual thrust levels, native systems determine actual available force, and torque correction/feedback use that actual result. Do not retain Phase 2 assumptions that vanilla owns the thrust request.

Introduce a simple allocator.

Start with constrained test cases:

- Pure translation
- Pure yaw
- Pure pitch
- Pure roll

Then combinations.

Success criteria:

- Desired translation is achieved within available authority
- Desired rotational torque is achieved where possible
- Net unwanted torque is minimized
- Thrust limits are respected

---

## Phase 5 - Assisted Flight Controller

Implement:

- Rotation-rate control
- Attitude stabilization
- Angular velocity cancellation
- Translation control
- Control prioritization

Pilot input should be converted to a desired wrench rather than directly forwarded to vanilla systems.

---

## Phase 6 - Newtonian / Modifier Modes

Implement:

- Shift modifier behavior
- Ctrl modifier behavior
- Raw/Newtonian mode
- Runtime mode switching
- Configurable bindings where practical

---

## Phase 7 - Control Authority and Damage Response

Add:

- Control-authority calculation
- Dynamic recomputation
- Damaged-thruster behavior
- Limited-axis detection
- Optional cockpit/debug display

---

## Phase 8 - Per-Grid Policy

Add:

```text
Auto
Force ACS
Force Vanilla
```

Ensure automatic policy still defaults to:

```text
Human -> ACS
AI/NPC -> Vanilla
```

---

## Phase 9 - ACS-Aware Autonomous Flight

Future scope only.

Potential goals:

- ACS-aware NPC ships
- ACS-aware autopilot
- Blueprint certification/design checks
- AI issuing desired wrench commands through the same allocator

Do not implement until player ACS is mature and stable.

---

# Performance Requirements

The allocator may run frequently, so avoid unnecessary expensive work.

Recommended model:

```text
Slow-changing geometry/state:
-> cached

Fast-changing state:
-> updated each relevant simulation tick
```

Cache:

- Thruster position relative to grid
- Thruster orientation
- Basic actuator geometry
- Grid block membership

Recalculate when necessary:

- COM changes materially
- Grid topology changes
- Thruster availability changes
- Subgrids change
- Damage changes actuator availability

Per-tick operations should focus on:

- input
- current effective thrust limits
- current velocity/angular velocity
- allocator solve
- thrust commands
- torque application

Performance should be tested on both small grids and large ships with many thrusters.

---

# Numerical Solver Requirements

The exact solver may be selected after prototyping.

Candidate approaches include:

- Bounded least squares
- Non-negative least squares
- Weighted least squares
- Small quadratic optimization
- Iterative heuristic allocator

Requirements:

```text
0 <= thrust_i <= max_i
```

Support weighted priorities between translation and rotation.

Define and test the control-loop details alongside the solver:

- coordinate frame for requested and measured wrench values
- simulation timestep and update-order assumptions
- command ramp limits or actuator response model
- saturation feedback using actual applied force
- deadbands/hysteresis to prevent command chatter
- behavior during topology changes and temporary authority loss

Do not over-engineer the initial implementation.

A fast approximate allocator with stable behavior is preferable to an expensive academically optimal solver.

---

# Testing Strategy

Unit-test the math separately from Space Engineers integration.

Example tests:

## Symmetric pure translation

Expected:

```text
Net requested force achieved
Net torque ~= 0
```

## Symmetric pure yaw

Expected:

```text
Requested yaw torque achieved
Net translation ~= 0
```

## Single off-center thruster

Expected:

```text
Correct force
Correct torque direction
Correct torque magnitude
```

## Destroyed thruster

Expected:

```text
Solver returns best feasible solution
No invalid thrust values
Control authority decreases
```

## Impossible request

Expected:

```text
Stable degraded solution
No oscillatory/unbounded behavior
Priority rules respected
```

## Runtime toggle

Expected:

```text
ACS ON -> realistic behavior
ACS OFF -> vanilla behavior
No stale overrides
No residual torque injection
```

## Spike acceptance measurements

For the asymmetric test craft, record a baseline and ACS-on run using the same initial state. Define before testing:

- expected torque sign in grid/world coordinates
- tolerated error for predicted versus measured initial angular acceleration
- maximum enable/disable transition latency in simulation ticks
- observation window after disable in which no plugin torque or stale override may remain
- expected result after client/server replication when multiplayer is supported

---

# Important Scope Constraints

For initial development, do not attempt:

- Full replacement physics engine
- Custom fuel simulation
- Custom atmospheric thrust model
- Reimplementation of Havok
- ACS autonomous NPC flight
- Complex cockpit UI
- New custom gyro block
- Production multiplayer support until the Phase 0 physics-authority gate has established a server-authoritative path
- Subgrid-wide RCS control unless required for core feasibility

Keep the first implementation focused on proving realistic thruster-generated torque and player ACS control.

---

# Design Principles

## Preserve Native Systems Where Possible

Use Space Engineers for systems it already handles correctly.

Correct only the missing or unrealistic flight-control behavior.

## Avoid Global Breakage

Existing NPC ships must remain functional.

## Human Control Activates Realism

The player should experience realistic behavior immediately when taking control of a ship, even if that ship was originally NPC-built.

## Geometry Matters

Ship design should determine control behavior.

## Failure Should Be Physical

Damaged or poorly designed ships should lose control authority rather than being artificially normalized.

## Runtime Reversibility

ACS must be safely disengageable without restarting the game.

## Build Incrementally

Prove each layer before advancing.

---

# Definition of Initial Success

The project should be considered technically viable when all of the following are demonstrated:

1. Pulsar plugin loads reliably.
2. Human-controlled grid detection works.
3. Thruster geometry and effective thrust are accessible.
4. Grid COM is accessible.
5. Off-center thruster torque can be correctly calculated.
6. Torque can be applied reliably to the grid.
7. The feature can be toggled on/off in game.
8. NPC/autonomous ships remain unaffected.
9. Vanilla behavior returns cleanly when ACS is disabled.
10. Physics authority is verified for every environment claimed to be supported; otherwise support is explicitly limited to verified local environments.

Only after these conditions are met should development proceed to the full thruster allocator and attitude-control system.

---

# Immediate Codex Task

Begin with **Phase 0: Research / Reconnaissance**.

Do not implement the complete ACS yet.

Inspect the current Space Engineers internals and Pulsar plugin environment and produce a report covering:

1. The player ship-control input path.
2. Human vs AI active-controller detection.
3. `MyGridThrustSystem` behavior.
4. `MyThrust` state and individual command possibilities.
5. How actual/effective thrust can be obtained.
6. `MyGyroSystem` and vanilla torque-control flow.
7. `MyGridPhysics` torque/impulse capabilities.
8. Grid center-of-mass access.
9. Relevant simulation/update ordering.
10. Suitable Harmony/Pulsar hook points if direct access is insufficient.
11. Risks related to multiplayer, prediction, or server authority.
12. A recommended implementation path for the Phase 1 instrumentation plugin.

Every referenced internal API, property, or method should be verified against the actual current game/decompiled code.

Do not invent class members.

Where multiple hook strategies are possible, compare them and recommend the least invasive option.
