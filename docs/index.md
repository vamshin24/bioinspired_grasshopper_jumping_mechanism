
# **Foldable Grasshopper-Inspired Walking Robot**

![Grasshopper Hind Legs](assets/prototype.jpeg){ width="500" }

### **Team**
- **Vamshi Narayana Babu** – vamshin24@asu.edu
- **Sameerjheet Singh Chabbra** – schhab18@asu.edu
- **Shawn Dimang** – shawn.dimang@asu.edu

### **Introduction**
This project explores the design and simulation of a bio-inspired walking robot based on the grasshopper's morphology. Originally conceived as a jumping mechanism, the design was pivoted to a walking gait to ensure stability and control using a four-bar linkage leg design.

The robot is constructed using a foldable cardboard technique with a five-layer lamination process, allowing for rapid prototyping and lightweight structure. We utilized MuJoCo for physics-based modeling and optimization, focusing on maximizing the distance traveled in 5 seconds as our primary performance metric.

**Project Pivot:**  
Our original goal was to study and optimize **jumping** performance of a grasshopper-inspired mechanism. During prototyping we observed more reliable and repeatable **walking behavior**, so we pivoted the project to focus on **walking locomotion** and use **distance covered in 5 seconds** as the primary performance metric.


### **Research Question**
How can a grasshopper-inspired four-bar linkage mechanism be optimized for stable walking using a foldable cardboard structure, and how well does the simulated performance in MuJoCo correlate with the physical prototype?


### **Background & Biomechanics**
Grasshoppers utilize a specialized leg structure that allows for powerful extension. We analyzed the kinematic chain of the grasshopper leg to adapt it into a four-bar linkage suitable for a walking gait driven by continuous servo rotation rather than explosive energy release.
![Grasshopper Robot](assets/femur.png){ width="500" }

### **Kinematics and Dynamics**
We modeled the system’s kinematics and dynamics using Python and Jacobian.

![Kinematics and Dynamics](assets/.ipynb){ width="500" }

### **Specifications Table**
| Parameter                               | Symbol        | Value (example) | Units    | Notes                                           |
|-----------------------------------------|---------------|-----------------|----------|-------------------------------------------------|
| Total robot mass                        | $m$           | ~0.045          | kg       | Measured including batteries and two servos     |
| Body length                             | $L_b$         | ~0.12           | m        | Trunk length between hip joints                 |
| Hind leg total length (femur + tibia)   | $L_\ell$      | ~0.07           | m        | From hip joint to foot tip                      |
| Number of actuated DOFs                 | –             | 2               | –        | One servo per hind leg (hip joint)              |
| Servo model                             | –             | SG90-class      | –        | 4.8–6 V micro servo, ~1.8 kg·cm stall torque    |
| Nominal servo angular range             | $\Delta\theta$| 60–90           | deg      | Commanded sweep for walking                     |
| Nominal gait frequency                  | $f$           | 1–3             | Hz       | Full back-and-forth motion of legs             |
| Effective joint stiffness (laminate)    | $k_\theta$    | 0.1–0.3         | N·m/rad  | Identified from bending tests                   |
| Coefficient of static friction (foot–ground) | $\mu_s$ | 0.5–0.7         | –        | Cardboard foot on lab surface                   |
| Primary performance metric              | –             | COM displacement| m        | Net forward distance along x-axis               |



### **Design Mechanism & Manufacturing**
- **Leg Design**: A four-bar linkage system mimics the femur-tibia articulation of a grasshopper in libreCAD. Added cuts, joints and hinges to the design.
- **Construction**: Built using a 5-layer lamination process (cardboard, adhesive, and flexible layers) to create robust yet foldable joints.
- **Actuation**: Driven by standard SG90 servos controlling the input crank of the linkage.
- **CAD Design**

![CAD](assets/ss3.png){ width="600" }

- **LibreCAD Design**

![libreCAD](assets/libre.png){ width="500" }

![libre1](assets/libre1.png){ width="100" }

- **Body/Trunk 5 Layer Design**
![5 Layers](assets/ss.png){ width="500"}

- **Hind Legs 5 Layer Design**
![5 Layers1](assets/5layer.png){ width="500"}

- [5 Layered Manufacturing Workflow for hind legs](notebooks/grass_manu/grass_manu.md)

- [5 Layered Manufacturing Workflow for main body(Trunk)](notebooks/grass_body/grass_body.md)

### **Parameter Sweep & Optimization**
We performed parameter sweeps on link material stiffness, damping, servo frequency and friction to identify the optimal configuration for speed and stability.

**Motor Parameter Identification:**

- We commanded sinusoidal or square-wave position trajectories at certain frequencies and recorded the actual angle using a printed protractor and video tracking.  

- The resulting angle–time data allowed us to estimate the effective maximum angular speed, the steady-state lag between command and motion, and qualitative saturation behavior at higher frequencies.

- [Motor Behavior: Parameter Identification](notebooks/Assignment5_RAS557/Assignment5_RAS557.md)

**Material Parameter Identification:**
We characterized the effective bending stiffness and damping of the laminated cardboard:

- **Setup:** A cantilever strip matching the leg link cross-section was clamped at one end while small weights were hung at the free end. Deflection under static load gave a force–displacement curve.

- **Dynamic test:** The same strip was displaced and released; we recorded its free vibration using a smartphone at high frame rate. Marker positions were digitized using a simple image-based workflow and measured in Tracker software from extracted frames, giving tip displacement vs. time.

- [Stiffness and dampening of the material: Parameter Identification](assets/beam1_FINAL.pdf)

**Friction Identification:**

- **Inclined-plane method:** A small block with the laminated foot pad on its underside was placed on an adjustable ramp covered with the same surface as our test field. The critical angle $\alpha_c$ at which the block started sliding gave $\mu_s \approx \tan \alpha_c$.

- **Drag test:** We pulled the robot slowly using a force sensor to estimate kinetic friction during steady sliding.

- [Friction and Spring: Parameter Identification](notebooks/New_assignment-1/New_assignment-1.md)

### **Modeling & Simulation (MuJoCo)**
We developed a full physics simulation in MuJoCo to test the stability and kinematics of the walking gait before physical assembly.

![mujoco](assets/ss1.png){ width="500"}

- [Model & Simulation Notebook](notebooks/model_and_simulation.ipynb) 
- **MuJoCo Model Walking Demonstration**:
<video controls width="500">
  <source src="assets/download.mp4" type="video/mp4">
</video>

- **Real-world Prototype:**
The trunk was assembled first, with servos mounted aft and their horns protruding through the side walls. Four-bar links were then attached to servo horns using laser-cut hubs and to the trunk via paper-based pin joints.

![real](assets/ss2.png){ width="500"}

### **Design Optimization**
We optimized our design on Angle parameter. The angles of the femur and tibia from one link to another was optimized with experimetal data. 

- [Design Optimization Notebook](notebooks/model_and_simulation.ipynb) 

### **Results**
The optimized design achieved a stable walking gait in simulation with a total distance of **X.XX meters in 5 seconds**. The physical prototype demonstrated similar kinematic behavior, with an approximate **XX% error** in stride length compared to the simulation.

### **Files & Downloads**
- [Final Report (PDF)](/RAS557_Final_Project_Report.docx)
- [CAD / DXFs](https://github.com/vamshin24/bioinspired_grasshopper_jumping_mechanism/tree/main/docs/assets/dxf)
- [MuJoCo XML & Control Code and other Jupyter Notebooks](https://github.com/vamshin24/bioinspired_grasshopper_jumping_mechanism/tree/main/docs/notebooks)
- **Walking Demonstration**:
<video controls width="500">
  <source src="assets/demo1.mp4" type="video/mp4">
</video>
- **Presentation**:
<!-- <video controls width="500">
  <source src="assets/demo1.mp4" type="video/mp4">
</video> -->

### **Course Info**
- **Course**: RAS 557 – Foldable Robotics
- **Semester**: Fall 2025
- **Instructor**: Prof. Daniel Aukes
