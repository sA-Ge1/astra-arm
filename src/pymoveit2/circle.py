import numpy as np
import matplotlib.pyplot as plt

# Create a circle with radius 25 cm
radius = 25
theta = np.linspace(0, 2*np.pi, 100)
x = radius * np.cos(theta)
y = radius * np.sin(theta)

# Create the plot
plt.figure(figsize=(8, 8))
plt.plot(x, y)
plt.grid(True)

# Set equal aspect ratio to make it circular
plt.axis('equal')

# Add labels and title
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.title('Circle with Radius 25 cm')

# Add origin point
plt.plot(0, 0, 'ro', label='Origin (0,0)')

# Add axis lines
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)

# Set axis limits
plt.xlim(-30, 30)
plt.ylim(-30, 30)

plt.legend()
plt.show()