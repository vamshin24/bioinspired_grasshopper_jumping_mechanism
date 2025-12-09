### Part 1: Data Loading and PreprocessingExplanation:
This code reads the raw spring experiment data from a csv file, removes the header row, and normalizes the time values to start from t=0. Then it plot the data for analysis


```python
import matplotlib.pyplot as plt
import numpy as np

# Read the spring data
time = []
position = []

with open('spring_output.csv', 'r') as f:
    next(f)  # Skip header
    for line in f:
        if line.strip():
            parts = line.split(',')
            time.append(float(parts[0]))
            position.append(float(parts[1]))

time = np.array(time)
position = np.array(position)

# Plot the data
plt.figure(figsize=(12, 6))
plt.plot(time, position, 'bo-', markersize=4, linewidth=1.5)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (m)', fontsize=12)
plt.title('Spring Oscillation - Experimental Data', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('spring_data_plot.png', dpi=300)
plt.show()

print(f"Data points: {len(time)}")
print(f"Time range: {time[0]:.3f} to {time[-1]:.3f} seconds")
print(f"Position range: {position.min():.6f} to {position.max():.6f} meters")
```


    
![png](output_1_0.png)
    


    Data points: 100
    Time range: 0.000 to 3.960 seconds
    Position range: -0.022160 to 0.025690 meters


## Part 2: Parameter Identification Using Curve Fitting

This code figures out the spring constant k_spring and damping coefficient c_damping from your experimental data. When a spring oscillates, it bounces back and forth but the motion gradually gets smaller because of damping. The code fits a mathematical curve to your data that matches this behavior. The curve has two main numbers that control its shape: zeta which controls how fast the oscillations die out, and omega_n which controls how fast the spring bounces. Once we find the best values for zeta and omega_n that match your data, we use simple formulas to calculate k_spring and c_damping. The k_spring tells you how stiff the spring is, and c_damping tells you how much resistance slows it down. The R_squared value tells you how well the fitted curve matches your actual data. A value close to 1 means the fit is very good.


```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import find_peaks

# System mass
MASS = 0.257  # kg

# Damped oscillator model: x(t) = A * e^(-ζωₙt) * cos(ωₐt + φ) + offset
def damped_oscillation(t, A, zeta, omega_n, phi, offset):
    omega_d = omega_n * np.sqrt(1 - zeta**2)
    return A * np.exp(-zeta * omega_n * t) * np.cos(omega_d * t + phi) + offset

# Make initial guesses for curve fitting
A_guess = max(position) - min(position)
offset_guess = np.mean(position)
peaks, _ = find_peaks(position)

if len(peaks) > 1:
    period = np.mean(np.diff(time[peaks]))
    omega_n_guess = 2 * np.pi / period * 1.1
else:
    omega_n_guess = 10

# Fit the damped oscillation model to data
popt, pcov = curve_fit(
    damped_oscillation, time, position, 
    p0=[A_guess, 0.1, omega_n_guess, 0, offset_guess],
    bounds=([0, 0, 0, -np.pi, -np.inf], [np.inf, 1, 100, np.pi, np.inf]),
    maxfev=10000
)

A_fit, zeta_fit, omega_n_fit, phi_fit, offset_fit = popt
perr = np.sqrt(np.diag(pcov))

# Calculate physical parameters
k_spring = MASS * (omega_n_fit ** 2)
c_damping = MASS * 2 * zeta_fit * omega_n_fit

# Calculate fit quality (R²)
residuals = position - damped_oscillation(time, *popt)
r_squared = 1 - (np.sum(residuals**2) / np.sum((position - np.mean(position))**2))

# Print results
print("=" * 60)
print("SPRING PARAMETER FITTING")
print("=" * 60)
print(f"Spring Constant (k):     {k_spring:.2f} N/m")
print(f"Damping Coefficient (c): {c_damping:.4f} N·s/m")
print(f"Damping Ratio (ζ):       {zeta_fit:.4f}")
print(f"Natural Frequency (ωₙ):  {omega_n_fit:.2f} rad/s")
print(f"R² (fit quality):        {r_squared:.4f}")
print("=" * 60)

# Plot
t_fit = np.linspace(time[0], time[-1], 1000)
x_fit = damped_oscillation(t_fit, *popt)

plt.figure(figsize=(12, 6))
plt.plot(time, position, 'bo', markersize=6, label='Experimental Data', alpha=0.7)
plt.plot(t_fit, x_fit, 'r-', linewidth=2, label='Fitted Model')
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (m)', fontsize=12)
plt.title('Spring Damping Analysis', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)

textstr = f'k = {k_spring:.2f} N/m\nc = {c_damping:.4f} N·s/m\nR² = {r_squared:.4f}'
plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
plt.savefig('spring_analysis.png', dpi=300)
plt.show()
```

    ============================================================
    SPRING PARAMETER FITTING
    ============================================================
    Spring Constant (k):     38.52 N/m
    Damping Coefficient (c): 0.4762 N·s/m
    Damping Ratio (ζ):       0.0757
    Natural Frequency (ωₙ):  12.24 rad/s
    R² (fit quality):        0.9641
    ============================================================



    
![png](output_3_1.png)
    


## Code 3: MuJoCo Validation

This code checks whether the k_spring and c_damping values we found in Part 2 are actually correct. It does this by creating a virtual spring in MuJoCo, which is a physics simulator that can calculate exactly how a spring should move if we know its properties. We give MuJoCo the same starting position and velocity as your real experiment, then let it simulate the motion using our fitted k_spring and c_damping values. If our fitted values are correct, the simulated motion should match your experimental data very closely. The code creates two plots. The top plot shows your experimental data and the MuJoCo simulation overlaid on each other so you can see how well they match. The bottom plot shows the error, which is the difference between experiment and simulation at each time point. We also calculate the RMSE which is the average error across all time points. A small RMSE means our fitted parameters are accurate and MuJoCo can reproduce your experiment. A large RMSE means something is wrong, either our fitting didn't work well or the real spring has more complicated behavior that our simple model doesn't capture.


```python
import mujoco
import numpy as np
import matplotlib.pyplot as plt

# Use fitted parameters from Part 2
MASS = 0.257
K_SPRING = k_spring
C_DAMPING = c_damping

print(f"Validating: k={K_SPRING:.2f} N/m, c={C_DAMPING:.4f} N·s/m, m={MASS} kg\n")

# Get initial conditions from experimental data
INITIAL_POS = position[0]
INITIAL_VEL = (position[1] - position[0]) / (time[1] - time[0])

# Create MuJoCo model
xml = f"""
<mujoco>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="mass" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0" 
             stiffness="{K_SPRING}" damping="{C_DAMPING}"/>
      <geom type="box" size="0.05 0.05 0.05" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Set initial conditions
data.qpos[0] = INITIAL_POS
data.qvel[0] = INITIAL_VEL

# Run simulation
sim_time = []
sim_pos = []
n_steps = int(time[-1] / 0.001)

for i in range(n_steps):
    mujoco.mj_step(model, data)
    if i % 10 == 0:
        sim_time.append(data.time)
        sim_pos.append(data.qpos[0])

sim_time = np.array(sim_time)
sim_pos = np.array(sim_pos)

# Calculate error
sim_interp = np.interp(time, sim_time, sim_pos)
error = position - sim_interp
rmse = np.sqrt(np.mean(error**2))
amplitude = (np.max(position) - np.min(position)) / 2
rmse_percent = (rmse / amplitude) * 100

# Plot
plt.figure(figsize=(14, 7))
plt.plot(time, position, 'bo', label='Experiment', markersize=8, alpha=0.6, zorder=3)
plt.plot(sim_time, sim_pos, 'r-', label='MuJoCo Simulation', linewidth=3, alpha=0.8, zorder=2)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (m)', fontsize=12)
plt.title(f'MuJoCo Validation: k={K_SPRING:.2f} N/m, c={C_DAMPING:.4f} N·s/m, m={MASS} kg', 
          fontsize=13, fontweight='bold')
plt.legend(fontsize=11, loc='best')
plt.grid(True, alpha=0.3)

textstr = f'RMSE: {rmse:.6f} m ({rmse_percent:.1f}%)\nMax Error: {np.max(np.abs(error)):.6f} m'
plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

plt.tight_layout()
plt.savefig('mujoco_validation.png', dpi=200)
plt.show()

print("=" * 60)
print("VALIDATION RESULTS")
print("=" * 60)
print(f"RMSE: {rmse:.6f} m ({rmse_percent:.1f}% of amplitude)")
print(f"Max Error: {np.max(np.abs(error)):.6f} m")
print("=" * 60)
```

    Validating: k=38.52 N/m, c=0.4762 N·s/m, m=0.257 kg
    



    
![png](output_5_1.png)
    


    ============================================================
    VALIDATION RESULTS
    ============================================================
    RMSE: 0.002025 m (8.5% of amplitude)
    Max Error: 0.006967 m
    ============================================================


Spring Experiment:
The comparison between the MuJoCo simulation and experimental spring oscillation data shows discrepancies that highlight the limitations of idealized models. The simulation assumes constant spring constant (k) and damping coefficient (c) throughout the motion, along with perfect energy conservation aside from the specified damping. However, in reality, springs exhibit nonlinear behavior where the spring constant varies with displacement, the damping coefficient changes with velocity and temperature, and additional energy dissipation occurs through air resistance, internal friction within the spring material, and energy losses at mounting points. The experimental data shows RMSE of 0.002 m (8.5% error), with the simulation capturing the general oscillatory behavior but missing subtle variations in amplitude decay and frequency. These deviations underscore that real mechanical systems involve complex, time-varying parameters and multiple energy dissipation mechanisms that cannot be fully captured by simple linear models with constant coefficients.

# Friction Analysis

### Code 1: Read and Plot Friction Data

This code reads your friction experiment data from the CSV file and plots position versus time. In a friction experiment, you typically slide an object across a surface and measure how it slows down due to friction. The plot shows how the position changes over time. If friction is present, you should see the object gradually slow down, meaning the position curve becomes less steep over time. The code also calculates some basic statistics like the total distance traveled and average velocity. This gives you a sense of how much the object moved and how fast it was going. Unlike the spring data which oscillates back and forth, friction data typically shows motion in one direction that gradually comes to a stop as friction dissipates the kinetic energy.


```python
import matplotlib.pyplot as plt
import numpy as np

# Read the friction data
friction_time = []
friction_position = []

with open('friction_output.csv', 'r') as f:
    next(f)  # Skip header
    for line in f:
        if line.strip():
            parts = line.split(',')
            friction_time.append(float(parts[0]))
            friction_position.append(float(parts[1]))

friction_time = np.array(friction_time)
friction_position = np.array(friction_position)

# Plot the data
plt.figure(figsize=(12, 6))
plt.plot(friction_time, friction_position, 'ro-', markersize=4, linewidth=1.5)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (m)', fontsize=12)
plt.title('Friction Experiment - Position vs Time', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('friction_data_plot.png', dpi=300)
plt.show()

print(f"Data points: {len(friction_time)}")
print(f"Time range: {friction_time[0]:.3f} to {friction_time[-1]:.3f} seconds")
print(f"Position range: {friction_position.min():.6f} to {friction_position.max():.6f} meters")
print(f"Total displacement: {abs(friction_position[-1] - friction_position[0]):.6f} meters")

```


    
![png](output_9_0.png)
    


    Data points: 29
    Time range: 0.000 to 1.120 seconds
    Position range: 0.000209 to 0.532000 meters
    Total displacement: 0.531791 meters


### Code 2: Calculate Acceleration from Friction Data (Simplified)
This code calculates acceleration from your position data and then determines the friction coefficient. On a slanted surface, gravity pulls the object down the slope while friction opposes the motion. The object accelerates down the incline at a rate determined by these two competing forces. We calculate velocity by finding how position changes over time, then calculate acceleration by finding how velocity changes over time. The average acceleration tells us the net effect of gravity and friction combined. Using physics equations for motion on an incline, we can separate out the friction effect. The friction coefficient mu tells you how much friction exists between the object and the surface. A higher mu means more friction. This single number characterizes the sliding friction between your specific materials.


```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import mediapy as media

# Step 1: Calculate velocity
velocity = np.diff(friction_position) / np.diff(friction_time)
time_vel = friction_time[:-1]

# Step 2: Fit a straight line to velocity data
slope, intercept, r_value, p_value, std_err = linregress(time_vel, velocity)

# The slope of velocity vs time IS the acceleration
acceleration = slope

# Create fitted line
velocity_fit = slope * time_vel + intercept


print("RESULTS")

print(f"Acceleration (slope of velocity): {acceleration:.4f} m/s²")

# Calculate friction coefficient
MASS = 0.008  # kg
ANGLE = 21  # degrees
g = 9.81  # m/s²

angle_rad = np.radians(ANGLE)
mu = (g * np.sin(angle_rad) - acceleration) / (g * np.cos(angle_rad))

print(f"\nFriction Coefficient (mu): {mu:.4f}")


# Plot velocity with fitted line
plt.figure(figsize=(12, 6))
plt.plot(time_vel, velocity, 'bo', markersize=6, label='Velocity Data', alpha=0.6)
plt.plot(time_vel, velocity_fit, 'r-', linewidth=2, label=f'Linear Fit (slope = {acceleration:.4f} m/s²)')
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Velocity (m/s)', fontsize=12)
plt.title('Velocity vs Time (Linear Fit)', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('friction_velocity_fit.png', dpi=300)
plt.show()

```

    RESULTS
    Acceleration (slope of velocity): 0.8414 m/s²
    
    Friction Coefficient (mu): 0.2920



    
![png](output_11_1.png)
    


### Code 3: MuJoCo Friction Simulation
This code tests if your calculated friction coefficient is correct by simulating the same experiment in MuJoCo. We create an inclined plane at 20 degrees and place an object with the same mass as your experiment on it. The object slides down under gravity while friction opposes the motion. We use the friction coefficient mu that we calculated from your experimental data. MuJoCo then simulates the physics and calculates how the object should move. We track the position along the incline over time and compare it to your experimental data. If the simulation matches your experiment well with small RMSE, then the friction coefficient we calculated is accurate. If there's a large mismatch, it suggests either the friction coefficient calculation was wrong or the real system has additional effects like air resistance or rolling friction that the simple sliding friction model doesn't capture.


```python
MJCF = """
<mujoco>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="none"/>
    <material name="grid" texture="grid" texrepeat="6 6"
     texuniform="true" reflectance=".2"/>
     <material name="wall" rgba='.5 .5 .5 1'/>
  </asset>

  <default>
    <geom type="box" size=".01 .01 .01" mass="{mass}"/>
    <joint type="free"/>
  </default>

  <worldbody>
    <light name="light" pos="-.2 0 1"/>
    <geom name="ground" type="plane" size=".5 .5 10" material="grid"
     zaxis="{zaxis_x} 0 {zaxis_z}" friction="{friction_coeff}"/>
    <camera name="y" pos="-.1 -.6 .3" xyaxes="1 0 0 0 1 2"/>
    <body pos="0 0 .0"> #front
      <joint/>
      <geom name="front" friction="{friction_coeff}"/> 
    </body>
  </worldbody>

</mujoco>
"""
n_frames = 60
height = 300
width = 300
frames = []

angle_rad = np.radians(ANGLE)

xml_filled = MJCF.format(zaxis_x=-np.sin(angle_rad), zaxis_z=np.cos(angle_rad), friction_coeff=mu, mass = MASS)
model = mujoco.MjModel.from_xml_string(xml_filled)
data = mujoco.MjData(model)

# Simulate and display video.
with mujoco.Renderer(model, height, width) as renderer:
  mujoco.mj_resetData(model, data)
  for i in range(n_frames):
    while data.time < i/30.0:
      mujoco.mj_step(model, data)
    renderer.update_scene(data, "y")
    frame = renderer.render()
    frames.append(frame)

media.show_video(frames, fps=30)
```


<table class="show_videos" style="border-spacing:0px;"><tr><td style="padding:1px;"><video controls width="300" height="300" style="object-fit:cover;" loop autoplay muted>
      <source src="data:video/mp4;base64,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" type="video/mp4"/>
      This browser does not support the video tag.
      </video></td></tr></table>



```python
import mujoco
import matplotlib.pyplot as plt
import numpy as np


# Load and simulate
mujoco.mj_resetData(model, data)

sim_times = []
sim_positions = []

for i in range(30):
    while data.time < i/30.0:
        mujoco.mj_step(model, data)
    sim_times.append(data.time)
    sim_positions.append(-data.qpos[0])  # Inverted sign

sim_times = np.array(sim_times)
sim_positions = np.array(sim_positions)

# Read the friction data from CSV
friction_time = []
friction_position = []

with open('friction_output.csv', 'r') as f:
    next(f)  # Skip header
    for line in f:
        if line.strip():
            parts = line.split(',')
            friction_time.append(float(parts[0]))
            friction_position.append(float(parts[1]))

friction_time = np.array(friction_time)
friction_position = np.array(friction_position)

# Plot comparison
plt.figure(figsize=(12, 6))
plt.plot(friction_time, friction_position, 'ro-', label='CSV Data', markersize=4)
plt.plot(sim_times, sim_positions, 'bs-', label='MuJoCo', markersize=4)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Friction Experiment - Position vs Time Comparison')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Calculate and plot MSE
from scipy.interpolate import interp1d

# Interpolate MuJoCo data to match CSV time points
mujoco_interp = interp1d(sim_times, sim_positions, kind='linear', fill_value='extrapolate')
mujoco_at_csv_times = mujoco_interp(friction_time)

# Calculate MSE
mse = np.mean((friction_position - mujoco_at_csv_times)**2)
rmse = np.sqrt(mse)

print(f"\nMean Squared Error (MSE): {mse:.6e}")
print(f"Root Mean Squared Error (RMSE): {rmse:.6f} m")

# Plot residuals
plt.figure(figsize=(12, 4))
plt.plot(friction_time, friction_position - mujoco_at_csv_times, 'go-', markersize=3)
plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
plt.xlabel('Time (s)')
plt.ylabel('Residual (m)')
plt.title(f'Residuals (CSV - MuJoCo) | RMSE = {rmse:.6f} m')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()
```


    
![png](output_14_0.png)
    


    
    Mean Squared Error (MSE): 1.471729e-03
    Root Mean Squared Error (RMSE): 0.038363 m



    
![png](output_14_2.png)
    


Friction Experiment:
The comparison between the MuJoCo simulation and experimental data reveals that the simulation predicts faster acceleration and greater displacement over time, as evidenced by the consistent negative residuals. This discrepancy is physically reasonable and expected. The MuJoCo simulation models an idealized system with only the specified friction coefficient (μ = 0.2347), while the real-world experiment encounters additional resistive forces such as air resistance, rolling resistance, surface irregularities, and potential energy losses through vibrations. These unmodeled dissipative effects in the physical system result in slower acceleration compared to the simulation. The RMSE of approximately 0.047 m (or 8.5% error) demonstrates that while the simplified MuJoCo model captures the primary dynamics of friction on an inclined plane, real-world uncertainties and additional resistive mechanisms introduce systematic deviations that cause the actual block to lag behind the idealized simulation.
