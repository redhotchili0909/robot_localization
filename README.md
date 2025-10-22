# CompRobo FA25: Robot Localization Project

*Chang Jun Park*\
*Ashley Yang*

## Project Overview

The goal of this project is to implement a particle filter to localize a Neato on a map, using Lidar sensor measurements to iteratively obtain probabilistic estimates of position and orientation until we have successfully converged on the Neato's position.

![Beginning Process](/assets/demo.gif)

[Full Demo Video](https://drive.google.com/file/d/1ALlIbmK2ntN1-Wz1V4m5cGHbPBzk0uzB/view?usp=sharing)

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

The overall goal of this step is to update our weights of our particles based essentially on how correct the Lidar scan data is when transposed onto each particle, and compared with our known map. We first convert all the laser scan data points into Cartesian x and y points, filtering out false scans of infinity or 0 as we go. With this filtered laser scan data, we go through every particle, and rotate then translate each scan point into the map frame via the particles' pose. 

Specifically, we rotate our laser scan coordinates by multiplying the inverse rotation matrix, which uses the angle heading of the particle, against the laser scan x and y coordinates; we then translate our scan coordinates by adding our particle coordinate vector.

$$\begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{particle}) & \sin(\theta_{particle}) \\ -\sin(\theta_{particle}) & \cos(\theta_{particle}) \end{bmatrix} \begin{bmatrix} x_{laser} \\ y_{laser} \end{bmatrix} + \begin{bmatrix} x_{particle} \\ y_{particle} \end{bmatrix}$$

Now, each of our laser scan data points has been transposed to be placed as if the particle was the actual robot, and we do this for every particle in our particle cloud.

With each of these laser scan data points, we then get its distance "error", which is calculated by the distance from the scan data point to the nearest obstacle according to our known map. Each of these distance errors are added to an overall distance error that we keep track of for each particle. After obtaining the overall distance error for a particle, we divide it by the amount of scans we transposed in order to get the mean distance error for that particle. 

After iterating through all the particles, we take the mean distance error for each particle, and use Gaussian normal distribution to calculate each particle weight according to where its error fell on the distribution; lower errors lead to higher weights and vice versa, where larger errors have a smooth increasing penalty.

Each particle weight is calculated with this equation (weights are normalized later), where sigma is our measurement noise variance, which has been arbitrarily set at 0.2 through testing:
$$
w_i = \exp\!\left(-\frac{(\text{error}_i)^2}{2\sigma^2}\right)
$$

We chose to stick with Gaussian normal distribution as it is simple and efficient; we also later circumvented its issue of susceptibility to outliers later on in the pose estimation step by picking the particle with the highest weight. 

If the total weights of the particles is below a set threshold of 0.0, we chose to make the weights uniform since this indicates there is no variance of error among the particles and therefore we can treat each particle the same for the next pass; if the total weights are above 0.0, we proceed with normalizing and assigning the weights to our particles.

### 4. Pose Estimation (`update_robot_pose`)

At this point we have a bunch of weighted particles, but we need to give the rest of the ROS system a single answer for the Neato's supposed location. We first averaged all the particles, but we discovered that the outlier particles sometimes gave weird results if we have multiple clusters of particles in different parts of the map. Instead, we decidede to just pick the particle with the highest weight - since this particle explains the data better than any other, we trust its pose. This maximum likelihood approach works well because it naturally handles situations where the particle filter isn't sure between a few different locations.

### 5. Resampling (`resample_particles`)

As the last step of the iteration, we resample the particles based on their calculated weights, in order to have an updated particle cloud for the next iteration. We make sure the distribution is normalized before resampling all of them. Initially, we planned to start with resampling all of them, and introducing random sampling as needed; in the end, it turned out that our particle filter still consistently works with complete weighted resampling. At the end, we reset the weights of each particle to be 0.1 to have a fresh start for the next iteration, and avoid math errors from having them reset to 0.

---

We only run steps 2-5 when the Neato has moved enough to make it worthwhile (0.1 meters or π/6 radians). This keeps the algorithm efficient while ensuring we track the Neato's motion accurately. The whole process repeats continuously, and over time the particle cloud converges on the robot's true location as we gather more sensor data.

## Results on Simulation

After successfully testing our particle filter implementation on the bag recording of the 1st floor of the MAC, we moved on to validate our algorithm in simulation. We used our previous teleoperation behavior code from the RoboBehaviors project to drive the Neato around in the Gazebo simulation of the Gauntlet environment.

As shown below, the particles converged correctly to the Neato's actual location with consistent accuracy. Even when starting with a dispersed particle cloud, the algorithm quickly honed in on the robot's true position as it moved through the environment.

![Gauntlet Teleoperation Test](/assets/gauntlet_teleop_pf.gif)

[Full Demo Video](https://drive.google.com/file/d/1bzSEoyizTYYjyOdvdCE2Ck4GvPnMfiXK/view?usp=sharing)

## Challenges Faced

The primary challenge we encountered was understanding the existing skeleton code structure. While the important algorithmic functions were left for us to implement, it took time to realize that there were plenty of helper functions already written that could significantly reduce our workload. We initially overlooked these utilities and started implementing functionality from scratch.

Additionally, we initially had a critical bug where our odometry updates weren't translating correctly to each of the particles. The mistake was applying the world-frame motion directly to each particle - we forgot that in our setup, each particle has its own random orientation, so we needed to apply an additional transform to accommodate that. This caused all particles to drift sideways together, which was confusing to debug. After fixing this transformation bug, the rest of the implementation became much more straightforward.

## Possible Future Improvements

Given more time, it could have been interesting to explore other weight distributions more in depth. Gaussian distribution worked very well for us, but tinkering around with other distributions such as Laplace distribution or a mixture of Gaussian and uniform distribution could potentially yield better results.

We could also experiment with adaptive methods for assigning weights, such as dynamically adjusting the measurement noise variance (sigma) based on the quality of laser scan data or implementing outlier rejection techniques to handle insufficient laser readings.

## Lessons Learned

### Ashley

When starting to implement a function, like updating particles with the laser scan data, it's helpful to not only understand the overall conceptual process but also have the middle-level details sketched out in pseudocode. Since the code encapsulates the fine grain details, what happened initially was that even though we had the big conceptual process understood early on (transpose the laser scan data onto the particles as if they're the Neato and calculate error), there were little things that would've made the functionality off if just from the first pass of coding. For example, making sure to rotate the laser scan data before translating, or filtering out the infinity value scan points, were things that weren't initially considered in the high level process, and had to be added in during multiple revisions. So, fleshing out the bridge between our conceptual model and the code first would've likely made our implementation process a bit more efficient.

### Jun

When debugging, it's important to think more broadly and try to understand the root cause rather than falling into a rabbit hole. For example, initially, for the aforementioned bug with the incorrect odometry transform update, we initially thought the issue was with our noise assignment because all the particles seemed to drift to the side together. We spent a considerable amount of time trying to understand why noise would cause that behavior. However, the answer was rather simple: our initial transform was incorrect. This experience taught us to step back and examine our initial assumptions when debugging, rather than diving deep into one suspected cause.