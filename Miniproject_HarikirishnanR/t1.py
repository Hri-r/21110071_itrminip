import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

# Defining the constants

L = 1.0   # length of arm 1
L_= 1.0   # length of arm 2

# Time parameters
t_max = 10.0     # total simulation time (s)
num_steps = 500

# Time array

time = np.linspace(0, t_max, num_steps)

def trajectory(t):
    # x=2*t*np.sin(t)/t_max
    # y=2*t*np.cos(t)/t_max
    x=1.9999*np.cos(t/1000)
    y=1.9999*np.sin(t/1000)

    return x, y

# Using Forward Kinematics for Computing the position of the end effector

def forward_kinematics(l, theta):
    x = l * np.cos(theta)
    y = l * np.sin(theta)
    return x,y

# Using Inverse Kinematics for Computing the angular output for each motor

def inverse_kinematics(x, y):
    theta=np.arccos( (x**2 + y**2-L**2 - L_**2)/(2*(L*L_)) )
    q1=np.arctan2(y,(x+1e-10))-np.arctan2( (L_*np.sin(theta)),(L+L_*np.cos(theta)) )
    q2=q1+theta
    return q1, q2

# Create animation

fig, ax = plt.subplots(figsize=(6, 6))  # Adjust the figure size to control the aspect ratio
ax.set_xlim(-1.5 * (L+L_), 1.5 * (L+L_))
ax.set_ylim(-1.5 * (L+L_), 1.5 * (L+L_))
line,  = ax.plot([], [], 'o-', lw=2)
line_,  = ax.plot([], [], 'o-', lw=2)


def init():
    line.set_data([], [])
    line_.set_data([],[])
    for t in time:
        x, y=trajectory(t)
        if((x**2 + y**2-L**2 - L_**2)/(2*(L*L_)))>(1+1e-10) or ((x**2 + y**2-L**2 - L_**2)/(2*(L*L_)))<-(1+1e-10):
            print("trajectory cannot be followed by the robot")
            exit(0)

    return line,line_,

def animate(i):
    # theta = states[i, 0]
    # theta_ = states_[i, 0]

    xd, yd=trajectory(i*t_max/num_steps)
    q1, q2=inverse_kinematics(xd, yd)
    x, y=forward_kinematics(L,q1)
    x_, y_=forward_kinematics(L_,q2)
    x_+=x
    y_+=y
    # print(xd, yd, x_, y_)
    # x_, y_=xd, yd
    line.set_data([0, x], [0, y])
    line_.set_data([x,x_], [y, y_])
    return line,line_,

ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=(t_max / num_steps) * 1000)

plt.xlabel('x')
plt.ylabel('y')
plt.title('2rM')
plt.grid()
plt.show()