
# Foldable Grasshopper-Inspired Walking Robot

![Grasshopper Robot](assets/prototype.jpeg){ width="300" }

### Team
- **Vamshi Narayana Babu** – vamshin24@asu.edu
- **Sameerjheet Singh Chabbra** – schhab18@asu.edu
- **Shawn Dimang** – shawn.dimang@asu.edu

### Introduction
This project explores the design and simulation of a bio-inspired walking robot based on the grasshopper's morphology. Originally conceived as a jumping mechanism, the design was pivoted to a walking gait to ensure stability and control using a four-bar linkage leg design.

The robot is constructed using a foldable cardboard technique with a five-layer lamination process, allowing for rapid prototyping and lightweight structure. We utilized MuJoCo for physics-based modeling and optimization, focusing on maximizing the distance traveled in 5 seconds as our primary performance metric.

### Research Question
How can a grasshopper-inspired four-bar linkage mechanism be optimized for stable walking using a foldable cardboard structure, and how well does the simulated performance in MuJoCo correlate with the physical prototype?

### Background & Biomechanics
Grasshoppers utilize a specialized leg structure that allows for powerful extension. We analyzed the kinematic chain of the grasshopper leg to adapt it into a four-bar linkage suitable for a walking gait driven by continuous servo rotation rather than explosive energy release.

### Mechanism & Manufacturing
- **Leg Design**: A four-bar linkage system mimics the femur-tibia articulation of a grasshopper.
- **Construction**: Built using a 5-layer lamination process (cardboard, adhesive, and flexible layers) to create robust yet foldable joints.
- **Actuation**: Driven by standard hobby servos controlling the input crank of the linkage.
- [5 Layered Manufacturing Workflow for hind legs](notebooks/grass_manu.ipynb)
- [5 Layered Manufacturing Workflow for main body(Trunk)](notebooks/grass_body.ipynb)

### Modeling & Simulation (MuJoCo)
We developed a full physics simulation in MuJoCo to test the stability and kinematics of the walking gait before physical assembly.
- [Model & Simulation Notebook](notebooks/model_and_simulation.ipynb)

### Parameter Sweep & Optimization
We performed parameter sweeps on link lengths and gait frequencies to identify the optimal configuration for speed and stability.
- [Assignment 5: Parameter Identification](notebooks/Assignment5_RAS557/Assignment5_RAS557.md)
- [Results Analysis Notebook](notebooks/results_analysis.ipynb)

### Results & Sim-to-Real
The optimized design achieved a stable walking gait in simulation with a total distance of **X.XX meters in 5 seconds**. The physical prototype demonstrated similar kinematic behavior, with an approximate **XX% error** in stride length compared to the simulation.

### Files & Downloads
- [Final Report (PDF)](RAS557_Final_Project_Report.docx)
- [CAD / DXFs](assets/cad_files/)
- [MuJoCo XML & Control Code](assets/mujoco_model/)
- [Photos & Videos](assets/media/)

### Course Info
- **Course**: RAS 557 – Foldable Robotics
- **Semester**: Fall 2025
- **Instructor**: Prof. Daniel Aukes
