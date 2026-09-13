# Realistic ACS client plugin

This is the Pulsar Legacy client-plugin component of Realistic ACS. It provides finite RCS authority, virtual ordinary-thruster torque allocation, persistent rotational inertia, rotational dampening, manual-thruster-override torque, and RCS gimbal animation for a locally controlled Space Engineers grid.

## Requirements

- Space Engineers 1 with Pulsar Legacy
- .NET Framework 4.8.1 Developer Pack
- A current .NET SDK

Game and Pulsar files are not included. They are resolved from the contributor's local installation at build time.

## Build and deployment

Build the `ClientPlugin/ClientPlugin.csproj` project for `net48`:

```powershell
dotnet build ClientPlugin\ClientPlugin.csproj -f net48
```

The project auto-detects common Steam and Pulsar paths. If required, create `Directory.Build.props.user` beside `Directory.Build.props` with local `Bin64` and `Pulsar` values; this machine-specific file is intentionally ignored.

A successful `net48` build deploys `plugin.dll` to `<Pulsar>\Legacy\Local\RealisticAcs`. Enable **Realistic ACS** in Pulsar before launching the game.

## Controls

- `/acs arm` and `/acs disarm` — master ACS enable/disable.
- `/acs status` — show active feature settings.
- `/acs inertia on|off` — enable or disable persistent rotational inertia.
- `/acs attitude arm|disarm` — enable or disable finite attitude authority.
- `/acs rotational-dampeners follow|on|off` — select rotational arrest behavior.

With rotational dampeners in `follow` mode, the ship's inertial dampeners choose whether released rotation is arrested. Holding Shift temporarily inverts that result.

## AlwaysOn grids and manual overrides

ACS normally manages only the player-controlled grid. To retain persistent inertia and manual override-torque behavior after leaving a player grid, place this setting in its qualifying cockpit's Custom Data:

```ini
[RealisticACS]
AlwaysOn=true
```

For a single-cockpit grid, that cockpit qualifies. For a multi-cockpit grid, it must be the vanilla **Main Cockpit**. The grid is registered only after a player controls it; ACS does not scan for or manage arbitrary/NPC grids. Registration is slow-validated and removed when the setting, qualifying cockpit, or grid is no longer valid.

Manual vanilla-thruster overrides are monitored without being changed. An off-centre overridden thruster creates proportional simulated torque. When rotational arrest is requested, ACS uses eligible virtual RCS and vanilla-thruster geometry to counter it; with arrest off, the rotation persists.

## Safety and diagnostics

ACS uses powered vanilla-thruster geometry as a virtual attitude-actuator model; it does not change their native thrust overrides. It applies simulated torque only when its selected actuator plan is a near-pure virtual force couple. Plans with a net-force residual above 2% of available actuator force are withheld rather than implying unbalanced translation. Hydrogen RCS demand overrides are restored when ACS is disarmed, control changes grid, or the plugin unloads.

The buffered diagnostic log is written to `SpaceEngineers/UserData/RealisticAcs/realistic-acs.log` (with one rotated `.previous` file); it is capped at 1 MiB per file. It is not written beside the installed plugin.

## Scope

The current implementation applies local physics. It is intended for local testing only and is not multiplayer-safe. A multiplayer implementation must be server-authoritative and synchronized.

## Repository hygiene

Do not share or commit generated `bin`/`obj` folders, `Directory.Build.props.user`, logs, or a Pulsar deployment directory. The parent `.gitignore` excludes them.
