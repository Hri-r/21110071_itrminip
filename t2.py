import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from matplotlib.animation import FuncAnimation

# Defining the constants

L = 2.0   # length of arm 1
L_= 2.0   # length of arm 2
xd, yd=1,1
x_hit, y_hit= 2, 1
x, y=0, 0
hit=False
N=100 #normal force to be applied on the wall

# Time parameters
t_max = 10.0     # total simulation time (s)
num_steps = 500

# Time array

time = np.linspace(0, t_max, num_steps)

def trajectory(t):
    x=t
    y=2.5*t-4
    return x, y

# normal force components
N_x=-N/np.sqrt(2)
N_y=-N/np.sqrt(2)


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


# Initial conditions

theta0 = np.pi / 4  # initial angle (radians)
omega0 = 0.0       # initial angular velocity (rad/s)
# initial_state = [theta0, omega0]

# Create animation

fig, ax = plt.subplots(figsize=(6, 6))  # Adjust the figure size to control the aspect ratio
ax.set_xlim(-1.5 * (L+L_), 1.5 * (L+L_))
ax.set_ylim(-1.5 * (L+L_), 1.5 * (L+L_))

line,  = ax.plot([], [], 'o-', lw=2)
line_,  = ax.plot([], [], 'o-', lw=2)

# wall
wall,  = ax.plot([], [], 'o-', lw=4)
wall.set_data([0,3], [3, 0])


def init():
    line.set_data([], [])
    line_.set_data([],[])
    return line,line_,

def find_torque(nx, ny, q1, q2):
    N=[[nx],
       [ny]]
    T=np.matmul([[-L*np.sin(q1), L*np.cos(q1)],[-L_*np.sin(q2), L_*np.cos(q2)]], N)
    return T

def animate(i):
    global hit
    xd, yd = trajectory(i * t_max / num_steps)
    q1, q2 = inverse_kinematics(xd, yd)
    x, y = forward_kinematics(L, q1)
    x_, y_ = forward_kinematics(L_, q2)
    x_ += x
    y_ += y

    if not hit and y_>= -x_+3 :
        hit = True
    
    if hit:
        x_, y_=x_hit, y_hit
        q1, q2=inverse_kinematics(x_hit, y_hit)
        x,y=forward_kinematics(L, q1)
        T=find_torque(N_x, N_y, q1, q2)
        print(f"Torque applied at joint 1 and joint 2 are T1={T[0][0]} and T2={T[1][0]} respectively.\nApplied normal force is 100N")
        ani.event_source.stop()

    line.set_data([0, x], [0, y])
    line_.set_data([x, x_], [y, y_])
    return line, line_

ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=(t_max / num_steps) * 1000)
plt.xlabel('x')
plt.ylabel('y')
plt.title('2rM')
plt.grid()
plt.show()