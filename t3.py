import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from matplotlib.animation import FuncAnimation

t_max=100
l1, l2=4, 4
m1, m2=1, 1
num_steps=500
time = np.linspace(0, t_max, num_steps)
# xd, yd=4, -5
xd, yd=0,0
xi, yi=2,2
k=10

def Tscalc(q1, q2):
    F=[[k*l1*np.cos(q1)+ k*l2*np.cos(q2)], [k*l1*np.sin(q1)+ k*l2*np.sin(q2)]]
    Ts=np.matmul([[-l1*np.sin(q1), l1*np.cos(q1)],[-l2*np.sin(q2), l2*np.cos(q2)]], F)
    return Ts

def trajectory(t):

    x=7.9*np.cos(t/1000)
    y=7.9*np.sin(t/1000)

    return x, y

def forward_kinematics(l, theta):
    x = l * np.cos(theta)
    y = l * np.sin(theta)
    return x,y

def inverse_kinematics(x, y):
    theta=np.arccos( (x**2 + y**2-l1**2 - l2**2)/(2*(l1*l2)) )
    q1=np.arctan2(y,(x+1e-10))-np.arctan2( (l2*np.sin(theta)),(l1+l2*np.cos(theta)) )
    q2=q1+theta
    return q1, q2

def forward_kinematics(l, theta):
    x = l * np.cos(theta)
    y = l * np.sin(theta)
    return x,y

q1i, q2i=inverse_kinematics(xi, yi)
q1f, q2f=inverse_kinematics(xd, yd)
init=[0, 0, q1i, q2i]

def dynamics(state, t):
    q1t,q2t, q1, q2=state
    mat=[[(m1*l1**2)/3 + m2*l1**2, 0.5*m2*l1*l2*np.cos(q2-q1)],
         [0.5*m2*l1*l2*np.cos(q2-q1), (m2*l2**2)/3 + 0.25*m2*l2**2]]

    Ts=Tscalc(q1,q2)

    t2=-Ts[1][0]
    t1=-Ts[0][0]-0.7*t2
        
    mat2=[[t1+0.5*l1*l2*q2t*(q2t-q1t)*np.sin(q2-q1)],
          [t2+0.5*l1*l2*q1t*(q2t-q1t)*np.sin(q2-q1)]]
    qtt=np.matmul(np.linalg.inv(mat), mat2)

    return [qtt[0][0], qtt[1][0], q1t, q2t]

states = odeint(dynamics, init, time)

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-1.5 * (l1+l2), 1.5 * (l1+l2))
ax.set_ylim(-1.5 * (l1+l2), 1.5 * (l2+l2))
line,  = ax.plot([], [], 'o-', lw=2)
line_,  = ax.plot([], [], 'o-', lw=2)
final1,  = ax.plot([], [], 'o-', lw=0.1)
final2,  = ax.plot([], [], 'o-', lw=0.1)

def init():
    line.set_data([], [])
    line_.set_data([],[])
    return line,line_,

def animate(i):
    q1=states[i, 2]
    q2=states[i, 3]

    x, y=forward_kinematics(l1,q1)
    xf, yf=forward_kinematics(l1,q1f)
    xf_, yf_=forward_kinematics(l2,q2f)
    xf_+=xf
    yf_+=yf

    x_, y_=forward_kinematics(l2,q2)
    x_+=x
    y_+=y
    line.set_data([0, x], [0, y])
    line_.set_data([x,x_], [y, y_])
    
    return line,line_,final1, final2

ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=(t_max / num_steps) * 1000)

plt.xlabel('x')
plt.ylabel('y')
plt.title('2rM')
plt.plot(4,5)
plt.grid()
plt.show()