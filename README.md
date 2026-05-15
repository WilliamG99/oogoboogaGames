# Real-Time Fluid Simulation using Smoothed-Particle Hydrodynamics

**Author:** William M. Garcia  
**Module:** CMP-6013Y – Final Year Project  
**Supervisor:** Prof. Stephen Laycock  
**University of East Anglia, 2024/25**

## About
This project implements a real-time 2D fluid simulation using Smoothed‑Particle Hydrodynamics (SPH).  
It is built in C on top of the [Ooga Booga](https://github.com/baldgg/oogabooga) engine, a custom game-development framework that provides rendering via DirectX 11, input handling, and a minimal standard library.

The simulation supports:
- Up to several thousand interacting fluid particles.
- Real‑time parameter tuning through an on‑screen UI.
- Mouse attraction (left button) and repulsion (right button).
- Spatial hashing (cell‑linked list) for efficient neighbour search.

## Requirements
- **Windows 10/11 (64‑bit)**
- **Visual Studio Build Tools 2022** with the **Desktop Development with C++** workload  
  - Ensure `MSVC v143 - VS 2022 C++ x64/x86 build tools` and `Windows 11 SDK` are selected during installation.
- **LLVM** (Clang) – version 18.1.7 or later  
  - Download from [LLVM releases](https://github.com/llvm/llvm-project/releases) and check **"Add to PATH"** during installation.
- **Visual Studio Code** with the **C/C++** extension installed.
- **Git** (to clone the repository, if not already present).

## Folder Structure
/
├── src/ # Main simulation source code
├── build/ # (Generated) build output and executable
├── oogabooga.code-workspace # VS Code workspace file
├── README.md # This file
└── ...

## How to Build and Run

1. **Open the project in VS Code**  
   Double‑click `oogabooga.code-workspace` or open VS Code → File → Open Workspace from File… and select it.

2. **Install the C/C++ extension** (if not already installed)  
   In VS Code, go to the Extensions view (`Ctrl+Shift+X`) and install **C/C++** by Microsoft.

3. **Run the Build Task**  
   - Press `Ctrl+Shift+P` to open the command palette.  
   - Type **`Run Build Task`** and select it.  
   - The build process will compile the engine and the simulation; a `build` folder will appear.

4. **Launch the simulation**  
   - After a successful build, press `F3` to start the application with the MSVC Debugger.  
   - Alternatively, run the executable found inside the `build` folder directly.

## Controls

| Input                         | Action                                                          |
|-------------------------------|-----------------------------------------------------------------|
| **Left mouse button** (hold)  | Attract particles toward the cursor                             |
| **Right mouse button** (hold) | Repel particles away from the cursor                            |
| **Escape**                    | Quit the simulation                                             |
| **On‑screen UI panel**        | Click a parameter field, type a value, press Enter to update it |
|-------------------------------|-----------------------------------------------------------------|

The UI panel allows real‑time adjustment of:
- Gravity
- Smoothing radius
- Particle mass
- Rest density
- Pressure / near‑pressure constants
- Viscosity
- Interaction radius / strength

## Note for the Marker
The simulation source code is the file that was originally shared in the conversation. All dependencies (the Ooga Booga engine) are included in the project folder. The build process automatically fetches and compiles everything needed. If you encounter any issues, please ensure the Visual Studio Build Tools and LLVM are correctly installed and that the `PATH` environment variable includes `clang`.

---

Enjoy the fluid dynamics!
