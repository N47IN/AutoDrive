import pandas as pd
import matplotlib.pyplot as plt
import scienceplots

plt.style.use('science')

data = pd.read_csv("/home/navin/colcon_ws/src/vehicles/controllers/laguerre/controllers/host_rev_rev/5_2_0.5.csv")
a = data['Sample time'].values
b = data['Accel'].values
c = data['Speed'].values
e = data['separation'].values

plt.plot(a, b)

plt.xlabel("Time")
plt.grid()
plt.ylabel("Speed")
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
 
