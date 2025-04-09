import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

class BrownianRobot:
    def __init__(self, arena_size=10.0, speed=0.2):
        self.arena_size = arena_size
        self.position = np.array([0.0, 0.0])  # start at center (0,0)
        self.speed = speed
        self.direction = np.array([1.0, 0.0])  # initial direction: to the right
        self.rotating = False   # indicates if currently in a rotation sequence
        self.rotation_frames = 0  # counts how many rotation steps remain
        self.rotation_angle_per_frame = 0.0  # rotation increment (radians) for each frame

    def update(self):
        """
        Update the robot's state. If the robot is rotating, update its heading;
        otherwise, move it forward. Check for arena boundary collisions.
        """
        if self.rotating:
            # Continue rotating in small increments
            if self.rotation_frames > 0:
                self.direction = self._rotate_vector(self.direction, self.rotation_angle_per_frame)
                self.rotation_frames -= 1
            else:
                # Rotation complete; resume moving straight
                self.rotating = False
        else:
            # Move forward in the current direction
            self.position += self.direction * self.speed
            # Check if the robot has collided with the arena boundaries
            if np.abs(self.position[0]) >= self.arena_size or np.abs(self.position[1]) >= self.arena_size:
                self._handle_collision()

    def _handle_collision(self):
        """
        Handle collision with the boundary:
         - Keep the position within the arena limits.
         - Set a random rotation (both angle and duration) so the robot turns before moving again.
        """
        # Ensure the position is within the bounds
        self.position = np.clip(self.position, -self.arena_size, self.arena_size)
        # Set up a rotation: choose a random total rotation angle between 90° and 180°
        total_rotation_angle = np.radians(random.uniform(90, 180))
        # Randomly decide the rotation direction (clockwise or anticlockwise)
        if random.random() < 0.5:
            total_rotation_angle = -total_rotation_angle
        # Choose a random number of frames for the rotation (between 5 and 20)
        self.rotation_frames = random.randint(5, 20)
        self.rotation_angle_per_frame = total_rotation_angle / self.rotation_frames
        self.rotating = True

    def _rotate_vector(self, vec, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return R.dot(vec)

def run_simulation():
    # Define arena parameters
    arena_size = 10.0 
    robot = BrownianRobot(arena_size=arena_size, speed=0.2)
    
    # Set up the figure and axis for animation
    fig, ax = plt.subplots()
    ax.set_xlim(-arena_size, arena_size)
    ax.set_ylim(-arena_size, arena_size)
    ax.set_aspect('equal')
    ax.set_title("Brownian Motion Simulation")
    
    # Plot objects: a point for the robot and a line for its trajectory
    robot_point, = ax.plot([], [], 'bo', markersize=8)
    traj_line, = ax.plot([], [], 'r-', linewidth=1)
    trajectory = [robot.position.copy()]

    def init():
        robot_point.set_data([], [])
        traj_line.set_data([], [])
        return robot_point, traj_line

    def animate(frame):
        # Update robot state and record its position for the trajectory trail
        robot.update()
        trajectory.append(robot.position.copy())
        robot_point.set_data([robot.position[0]], [robot.position[1]])
        xs, ys = zip(*trajectory)
        traj_line.set_data(xs, ys)
        return robot_point, traj_line

    # Create the animation object using matplotlib's FuncAnimation
    ani = animation.FuncAnimation(
        fig, animate, frames=1000, init_func=init, interval=50, blit=True
    )
    
    plt.show()

if __name__ == "__main__":
    run_simulation()