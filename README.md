# Isaac Sim Aerodynamics Extension

An Nvidia Omniverse / Isaac Sim extension that provides high-fidelity aerodynamic simulation capabilities for rigid bodies (e.g., ROVs, UAVs). This extension integrates seamlessly with `UsdPhysics` to simulate realistic aerodynamic forces including drag, lift, and stability moments.

## Features

- **High-Fidelity Aerodynamics**: Accurately calculates aerodynamic forces (drag, lift) and stability moments.
- **Center of Mass Integration**: Automatically uses the body's true Center of Mass (CoM) via `UsdPhysics.MassAPI` for applying physical aerodynamic forces.
- **Context Menu Integration**: Easily configure aerodynamic properties on any chosen prim using a custom UI integrated right into the Isaac Sim context menu.
- **Customizable Coefficients**: Easily calibrate aerodynamic coefficients based on empirical or CFD technical analysis reports.

## Installation

1. Clone this repository into your chosen Omniverse / Isaac Sim extension directory (e.g., `extsUser/`).
2. Open Isaac Sim.
3. Go to **Window** -> **Extensions**.
4. Search for **Aerodynamics** and click the toggle switch to enable it. (Ensure the cloned directory is included in your Extension Search Paths).

## Usage

1. Add a Rigid Body with `UsdPhysics` properties to your scene.
2. Right-click the object in the viewport or Stage panel.
3. Select **Add Aerodynamics Properties** (or use the Aerodynamics widget in the UI) to attach the aerodynamic script.
4. Adjust the aerodynamic coefficients (drag, lift, reference area, etc.) in the Property window based on your vehicle's specifications.
5. Press **Play** to start the simulation and observe the aerodynamic forces taking effect!

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
