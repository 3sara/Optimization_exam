import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def calculate_position(link1_length, link2_length, theta1, theta2):
    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    
    # Calculate end effector position
    x = link1_length * math.cos(theta1_rad) + \
        link2_length * math.cos(theta1_rad + theta2_rad)
    y = link1_length * math.sin(theta1_rad) + \
        link2_length * math.sin(theta1_rad + theta2_rad)
    
    return x, y

def animate_arm(angle1, angle2, duration=50):
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'o-', lw=2)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    
    def update(frame):
        x, y = calculate_position(5, 5, angle1 + frame/duration*10, angle2)
        line.set_data([0, x], [0, y])
        return line,
    
    anim = FuncAnimation(fig, update, frames=duration, blit=False, interval=20)
    plt.show()

# Main program
print("Enter the length of the first link:")
link1 = float(input())
print("Enter the length of the second link:")
link2 = float(input())

print("Enter the initial angle of the first link (degrees):")
angle1_init = float(input())
print("Enter the initial angle of the second link (degrees):")
angle2_init = float(input())

animate_arm(angle1_init, angle2_init)
