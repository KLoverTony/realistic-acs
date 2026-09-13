# Realistic ACS

Realistic ACS is a Space Engineers 1 client-side prototype for finite, damage-sensitive attitude control. It combines ordinary-thruster leverage with purpose-built RCS clusters, keeps rotational inertia when rotation is not being arrested, and follows the ship's inertial-dampener state for rotational dampening.

This repository contains two independently installable parts:

| Folder | Contents |
| --- | --- |
| `acs-client-plugin` | The Pulsar client plugin that supplies attitude-control logic and RCS animation. |
| `acs-rcs-block-mod` | The local Space Engineers content mod containing the ion and hydrogen RCS blocks, models, and icon. |

## Scope and limitations

This is a local, client-side prototype. Its physics effects are applied by the local Pulsar plugin and are not multiplayer-safe or authoritative. A multiplayer release requires a server-authoritative implementation and explicit synchronization.

## Requirements

- Space Engineers 1
- Pulsar Legacy for the client plugin
- .NET Framework 4.8.1 Developer Pack and a current .NET SDK to build the plugin

No Space Engineers, Pulsar, game assemblies, or compiled plugin dependencies are included here. The build resolves game assemblies from the contributor's own installation.

## Installation for local testing

1. Build `acs-client-plugin/ClientPlugin/ClientPlugin.csproj` for `net48`. The project can auto-detect common Steam and Pulsar locations; use `Directory.Build.props.user` for local overrides.
2. Enable the resulting `Realistic ACS` plugin in Pulsar Legacy.
3. Copy or junction-link `acs-rcs-block-mod` into `%AppData%\SpaceEngineers\Mods`, enable it in a world, and restart/reload the world after asset changes.
4. Turn vanilla gyros off on a test grid. Place powered ACS RCS blocks and control the grid from a cockpit.

## Player controls

- `/acs arm` and `/acs disarm` enable or disable the master ACS attitude system.
- `/acs inertia on|off` controls persistent rotational inertia.
- `/acs attitude arm|disarm` controls finite attitude authority without changing inertia.
- `/acs rotational-dampeners follow|on|off` sets rotational arrest behavior. `follow` uses the ship's inertial dampeners.
- Hold Shift to momentarily invert rotational arrest: drift while dampeners are on, or arrest rotation while they are off.
- `/acs status` reports active settings.

For an unattended player grid, add the following to the qualifying cockpit's Custom Data:

```ini
[RealisticACS]
AlwaysOn=true
```

On a multi-cockpit grid, the qualifying cockpit must be the vanilla **Main Cockpit**. ACS registers such a grid only after a player has controlled it; it does not scan or manage arbitrary/NPC grids. While registered and unattended, persistent rotational inertia and manual vanilla-thruster override torque remain active; there is no invented pilot rotation input.

## Manual thruster overrides

An enabled, working, powered vanilla thruster with a nonzero manual override produces simulated torque according to its current thrust and lever arm about the grid's center of mass. With inertial dampeners requesting rotational arrest, ACS uses all eligible virtual RCS/vanilla authority to counter that disturbance; any unavailable counter-authority remains as residual rotation. With dampeners off, the override-induced rotation persists. ACS monitors these overrides but never writes or changes their native values.

## RCS block ratings

| Block | Nominal ACS force | Resource model |
| --- | ---: | --- |
| ACS RCS Cluster (ion) | 50 kN | Electrical only |
| ACS Hydrogen RCS Cluster | 150 kN | Conveyor-supplied hydrogen |

The plugin allocates each working pod's finite force according to its position relative to the grid's center of mass. It also uses the geometry and effective capacity of working, powered ordinary thrusters as a **virtual attitude-actuator model**. The controller accepts only balanced virtual force couples, then applies their simulated torque; it does not override, fire, or consume resources through the ordinary thrusters themselves. This preserves vanilla translation control while making ordinary-thruster placement relevant to ACS authority. Manual overrides are the exception only in that ACS observes their existing thrust to model their lever-arm torque; it still does not modify them.

## Sharing and development hygiene

Do not commit `bin/`, `obj/`, local `Directory.Build.props.user` files, logs, or deployed Pulsar folders. The included `.gitignore` covers these items.

## License

Copyright (c) 2026 KLoverTony. Realistic ACS, including its source, definitions, models, and textures in this repository, is licensed under the [MIT License](LICENSE). See [asset provenance](ASSET_PROVENANCE.md) for the custom `.mwm` and `.dds` assets. Space Engineers and Pulsar are not included and remain subject to their respective owners' terms.
