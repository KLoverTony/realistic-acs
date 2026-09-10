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

## RCS block ratings

| Block | Nominal ACS force | Resource model |
| --- | ---: | --- |
| ACS RCS Cluster (ion) | 50 kN | Electrical only |
| ACS Hydrogen RCS Cluster | 150 kN | Conveyor-supplied hydrogen |

The plugin allocates each working pod's finite force according to its position relative to the grid's center of mass. Ordinary thrusters are also considered for available torque using current effective thrust. The plugin does not override their translation thrust.

## Sharing and development hygiene

Do not commit `bin/`, `obj/`, local `Directory.Build.props.user` files, logs, or deployed Pulsar folders. The included `.gitignore` covers these items.

