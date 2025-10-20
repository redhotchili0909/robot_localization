# CompRobo FA25: Robot Localization Project

*Chang Jun Park*\
*Ashley Yang*

## Project Overview

The goal of this project is to implement a particle filter to localize a Neato on a map, using Lidar sensor measurements to iteratively obtain probabilistic estimates of position and orientation until we have successfully converged on the Neato's position.

[Full Demo Video](https://drive.google.com/file/d/1ALlIbmK2ntN1-Wz1V4m5cGHbPBzk0uzB/view?usp=sharing)

![Beginning Process](/assets/demo.gif)

We made the decision to stick with a normal Gaussian distribution to determine the weights of our particles.

## Methodology

Our particle filter algorithm consists of 5 main steps that are executed iteratively. 

-  Initialize a set of particles via random sampling
-   Update the particles using data from odometry
-   Reweight the particles based on their compatibility with the laser scan
-   Resample with replacement a new set of particles with probability proportional to their weights.
-   Update your estimate of the Neato’s pose given the new particles. Update the map to odom transform.


Each step corresponds to a specific function in our implementation, and together they implement the standard predict-update-resample cycle seen commonly in most particle filter algorithms.

### 1. Initializing the Particle Cloud (`initialize_particle_cloud`)

This is where we set up our initial guesses about where the Neato might be. We create 300 particles scattered around what we think is the Neato's starting position. If we don't have a good initial guess, we just use whatever the odometry tells us and work from there. But since we know that initial estimate probably isn't perfect, we spread the particles out a bit by sampling from a Gaussian distribution. We use a standard deviation of 0.5 meters for the x and y positions and π/8 radians for the orientation, which gives us a reasonable spread without going completely out-of-bounds.

Each particle starts with the same weight (1/300), basically saying that since we have no idea which particle is correct, we set them all equally likely for now." This creates our initial cloud of hypotheses about where the Neato could be, which captures our uncertainty about the true starting position of the Neato.

### 2. Motion Update (`update_particles_with_odom`)

This step moves our particles based on how the Neato actually moved, using odometry data to see the change in x, y, and rotation since the last update. However, since odometry gives us movement in world coordinates, we need a two-step transformation process to properly apply this motion to each particle.

The reason we don't directly apply world-frame motion to particles is that the same world movement represents different body-frame movements depending on the robot's orientation.

By first converting to the body frame, we get a motion description that's independent of orientation. Then we can apply this body-frame motion to each particle according to that particle's individual orientation, ensuring physical consistency.

Given the motion in world coordinates $(\Delta x_{world}, \Delta y_{world}, \Delta \theta)$ and the robot's previous orientation $\theta_{old}$, we apply the inverse rotation matrix:

$$\begin{bmatrix} \Delta x_{body} \\ \Delta y_{body} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{old}) & \sin(\theta_{old}) \\ -\sin(\theta_{old}) & \cos(\theta_{old}) \end{bmatrix} \begin{bmatrix} \Delta x_{world} \\ \Delta y_{world} \end{bmatrix}$$

For each particle with orientation $\theta_p$, we transform the body-frame motion back to the map frame:

$$\begin{bmatrix} \Delta x_{particle} \\ \Delta y_{particle} \end{bmatrix} = \begin{bmatrix} \cos(\theta_p) & -\sin(\theta_p) \\ \sin(\theta_p) & \cos(\theta_p) \end{bmatrix} \begin{bmatrix} \Delta x_{body} \\ \Delta y_{body} \end{bmatrix}$$

Finally, we update each particle's position with Gaussian noise to account for odometry uncertainty:

$$\begin{align}
x_p^{new} &= x_p^{old} + \Delta x_{particle} + N_{x} \\
y_p^{new} &= y_p^{old} + \Delta y_{particle} + N_{y} \\
\theta_p^{new} &= \theta_p^{old} + \Delta \theta + N_{\theta}
\end{align}$$

The noise values we use are small (0.1 meters for position, π/90 radians for rotation) but important for keeping the particle cloud diverse and preventing all particles from clustering together too tightly. This accounts for real-world odometry inconsistencies like wheel slippage and encoder errors.

### 3. Sensor Update (`update_particles_with_laser`)

The overall goal of this step is to update our weights of our particles, essentially based on how correct the Lidar scan data is when transposed onto each particle compared with our known map. We first convert all the laser scan data points into Cartesian x and y points, filtering out false scans of infinity or 0 as we go. With this filtered laser scan data, we iterate through every particle, and rotate then translate each scan point into the map frame via the particles' pose. 

We rotate by multiplying the inverse rotation matrix against the laser coordinate vector.
$$\begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{particle}) & \sin(\theta_{particle}) \\ -\sin(\theta_{particle}) & \cos(\theta_{particle}) \end{bmatrix} \begin{bmatrix} x_{laser} \\ y_{laser} \end{bmatrix}$$

To translate, we just add the particle's x and y onto the rotated coordinates.

For each scan point that we transpose, we get its distance "error", adding it on to an overall distance error that we keep track of for each particle. After obtaining the overall distance error, we divide it by the amount of scans we transposed in order to get the mean distance error. 

We take the mean distance errors for each particle, and use Gaussian normal distribution to update each particle weights, normalizing the weights after. However, if the total weights of the particles is below a set threshold, we chose to make the weights uniform since it's not significant enough.

### 4. Pose Estimation (`update_robot_pose`)

At this point we have a bunch of weighted particles, but we need to give the rest of the ROS system a single answer for the Neato's supposed location. We first averaged all the particles, but we discovered that the outlier particles sometimes gave weird results if we have multiple clusters of particles in different parts of the map. Instead, we decidede to just pick the particle with the highest weight - since this particle explains the data better than any other, we trust its pose. This maximum likelihood approach works well because it naturally handles situations where the particle filter isn't sure between a few different locations.

### 5. Resampling (`resample_particles`)

As the last step of the iteration, we resample the particles based on their calculated weights, in order to have an updated particle cloud for the next iteration. We make sure the distribution is normalized before resampling all of them. At the end, we reset the weights of those particles to be 0.1 to have a fresh start for the next iteration, and avoid math errors from having them reset to 0.

---

We only run steps 2-5 when the Neato has moved enough to make it worthwhile (0.1 meters or π/6 radians). This keeps the algorithm efficient while ensuring we track the Neato's motion accurately. The whole process repeats continuously, and over time the particle cloud converges on the robot's true location as we gather more sensor data.

## Challenges Faced



## Possible Future Improvements

Given more time, it could have been interesting to explore other weight distributions more in depth.

## Lessons Learned