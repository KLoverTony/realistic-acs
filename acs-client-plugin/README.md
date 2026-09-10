# Realistic ACS client plugin

This is the Pulsar Legacy client-plugin component of Realistic ACS. It provides finite RCS authority, ordinary-thruster torque allocation, persistent rotational inertia, rotational dampening, and RCS gimbal animation for a locally controlled Space Engineers grid.

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

## Scope

The current implementation applies local physics. It is intended for local testing only and is not multiplayer-safe. A multiplayer implementation must be server-authoritative and synchronized.

## Repository hygiene

Do not share or commit generated `bin`/`obj` folders, `Directory.Build.props.user`, logs, or a Pulsar deployment directory. The parent `.gitignore` excludes them.
