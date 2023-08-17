import numpy as np
import matplotlib.pyplot as plt

# Defining the constants

L = 2.0   # length of arm 1
L_= 4.0   # length of arm 2

# Using Forward Kinematics for Computing the position of the end effector

def forward_kinematics(l, theta):
    x = l * np.cos(theta)
    y = l * np.sin(theta)
    return x,y

q1=np.linspace(35, 145, 1000)
q2=np.linspace(35, 145, 1000)

# Create animation

fig, ax = plt.subplots(figsize=(6, 6))  # Adjust the figure size to control the aspect ratio
ax.set_xlim(-1.5 * (L+L_), 1.5 * (L+L_))
ax.set_ylim(-1.5 * (L+L_), 1.5 * (L+L_))
        
X=[]
Y=[]

for i in q1:
    for j in q2:
        x, y=forward_kinematics(L,i*np.pi/180)
        x_, y_=forward_kinematics(L_,j*np.pi/180)
        x_+=x
        y_+=y
        X.append(x_)
        Y.append(y_)

plt.scatter(X, Y, s=0.5)
plt.xlabel('x')
plt.ylabel('y')
plt.title('2rM')
plt.grid()
plt.show()