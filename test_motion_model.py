import numpy as np
import matplotlib.pyplot as plt

# Define standard deviations for the normal distributions
std_pos = [0.25, 0.25, 0.03, 0.03]

# Generate a large number of particles
num_particles = 1

# Initialize particle positions (x, y, theta)
particles = np.zeros((num_particles, 3))
previous_observation = np.array([1.0, 1.0])
current_observation = np.array([3.0, 3.0])


# Generate random displacements based on the normal distributions
x_noise = np.random.normal(0, std_pos[0], num_particles)
y_noise = np.random.normal(0, std_pos[1], num_particles)
yaw_noise = np.random.normal(0, std_pos[3], num_particles)

# Apply the random displacements to the particles
particles[:, 0] += x_noise
particles[:, 1] += y_noise
particles[:, 2] += yaw_noise

# Check if the norm of the previous observation is not zero
if np.linalg.norm(previous_observation) != 0:
    avg_displacement = np.zeros(2)
    particles_before = particles.copy()
    for i in range(num_particles):
        p = particles[i]

        # Calculate displacement vector from previous readings
        dx = current_observation[0] - previous_observation[0]
        dy = current_observation[1] - previous_observation[1]
        avg_displacement = np.array([dx, dy])

        # Apply random noise to particle position and orientation
        delta_x = avg_displacement[0] + x_noise[i]
        delta_y = avg_displacement[1] + y_noise[i]
        delta_yaw = yaw_noise[i]

        particles[i, 0] += delta_x
        particles[i, 1] += delta_y
        particles[i, 2] += delta_yaw

# Plot the particle positions
plt.figure(figsize=(10, 10))
plt.scatter(particles[:, 0], particles[:, 1], c='brown', alpha=1, label='after')
plt.scatter(particles_before[:, 0], particles_before[:, 1], c='green', alpha=1, label='before')
plt.scatter(previous_observation[0], previous_observation[1], c='red', alpha=1, label='prev_obs')
plt.scatter(current_observation[0], current_observation[1], c='blue', alpha=1, label='curr_obs')
plt.title('Particle Positions After Applying Random Noise')
plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.grid(True)
plt.show()
