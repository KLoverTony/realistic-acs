# Large-grid RCS models

Each RCS variant has a root model plus azimuth and elevation subparts:

- Ion: `RCS_Pod_Main.mwm`, `RCS_Pod_Azimuth.mwm`, `RCS_Pod_Elevation.mwm`
- Hydrogen: `RCS_hydrogen_Pod_Main.mwm`, `RCS_hydrogen_Pod_Azimuth.mwm`, `RCS_hydrogen_Pod_Elevation.mwm`

The block SBC definitions reference the root models. The Realistic ACS plugin instantiates the subparts through their model dummies and animates them to visualize allocated RCS force. The block's bottom face is its attachment face.
