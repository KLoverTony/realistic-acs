# Realistic ACS - RCS Blocks

This is the content-mod half of Realistic ACS. It provides two buildable, powered large-grid RCS-cluster blocks:

| Grid | Subtype ID | Model path |
| --- | --- | --- |
| Large | `RealisticAcsRcsLarge` | `Models\Cubes\Large\RCS_Pod_Main.mwm` |
| Large | `RealisticAcsRcsHydrogenLarge` | `Models\Cubes\Large\RCS_hydrogen_Pod_Main.mwm` |

## Function

The ion block is a near-zero-force gyro-derived functional block, leaving native gyro authority negligible (`ForceMagnitude = 1`). The hydrogen block is a near-zero-force hydrogen-thruster definition: it supplies a conveyor-connected hydrogen path while retaining negligible native translation thrust. The Realistic ACS Pulsar plugin supplies the finite attitude authority: 50 kN per ion pod and 150 kN per hydrogen pod. Authority is additive per working pod and disappears if that pod is disabled or destroyed.

Both model sets contain a root, azimuth subpart, and elevation subpart. The plugin instantiates and animates the subparts. Both definitions use `Textures\GUI\Icons\Cubes\RCS_pod.dds` as their G-menu icon.

## Local installation

Copy or junction-link this folder into Space Engineers' local Mods directory, enable it for a world, and enable the Realistic ACS Pulsar plugin. Gimbals and finite authority are active only while the block is powered and working. Restart/reload the world after modifying models or textures.

## Design guardrail

Do not increase `ForceMagnitude`. That would restore uncontrolled vanilla gyro or thruster behavior and undermine the finite, damage-sensitive RCS model.
