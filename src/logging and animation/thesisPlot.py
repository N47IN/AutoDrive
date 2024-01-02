import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv("/home/navin/colcon_ws/src/vehicles/controllers/laguerre/controllers/host_rev_rev/10_1.25_1.csv")
a = data['Sample time'].values
b = data['Accel'].values
c = data['Speed'].values
e = data['separation'].values


fig, ax1 = plt.subplots()

color = 'tab:orange'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Acceleration', color=color)
ax1.plot(a, b, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax1.set_ylabel('Acceleration', fontsize=16)
ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
plt.xlabel('Time (s)',fontsize=16)
color = 'tab:blue'
ax1.tick_params(axis='both', which='major', labelsize=13)
ax2.tick_params(axis='both', which='major', labelsize=13)
ax2.set_ylabel('Separation', color=color, fontsize=16)  # we already handled the x-label with ax1
ax2.plot(a, e, color=color)
ax2.tick_params(axis='y', labelcolor=color)
plt.grid()
fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()

'''
 # Initialise the subplot function using number of rows and columns
figure, axis = plt.subplots(2, 2)
plt.rcParams['font.size'] = '10'
plt.subplots_adjust(left=0.08,
                    bottom=0.08,
                    right=0.95,
                    top=0.9,
                    wspace=0.211,
                    hspace=0.373)

axis[0, 0].plot(a, d,linewidth=1)
axis[0, 0].set_title("Speed v/ time")
axis[0, 1].plot(a, b,linewidth=1)
axis[0, 1].set_title("Acceleration v/ time") 
axis[1, 0].plot(a, e,linewidth=1)
axis[1, 0].set_title("Separation v/ time")
axis[1, 1].plot(a, d, color='purple',linewidth=1)
axis[1, 1].plot(a, b,linewidth=1)
axis[1, 1].plot(a, e, color='orange',linewidth=1)
axis[1, 1].set_title("v/ time")'''

plt.show()  
 
