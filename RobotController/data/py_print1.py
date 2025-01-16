import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import ipdb

MAX_POINTS = 2000
z_force_reference = 7
z_force_reference_max = 20
z_force_reference_min = 5
y_lower_lim = -10
y_upper_lim = 60
legends = ['x force', 'y force', 'z force']


def update_plot(frame):
    try:

        with open('/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/datalog.txt', 'r') as file:
            lines = file.readlines()

        if len(lines) > 0:

            last_line_values = lines[-1].strip().split()

            if len(last_line_values) != 13:
                lines = lines[:-1]
                
        
        lines = lines[-MAX_POINTS:]
        
        process_lines = []
        for line in lines:
            number = [float(num) for num in line.strip().split()[:-1]]
            process_lines.append(number)


        data = np.array(process_lines)

        ax.clear()






        ax.plot(data[:,0]/100, data[:,3], label = 'force feedback',color = 'b',linewidth = 4)
        contact_force =np.full(data.shape[0], z_force_reference)
        ax.plot(data[:,0]/100, contact_force, label = 'force reference',linestyle = '--',color = 'r',linewidth = 4)
        





        ax.set_xlabel('Time',fontsize = 20)
        ax.set_ylabel('Force Vaule in Z axis of Tool Frame',fontsize = 20)
        ax.set_ylim(y_lower_lim,y_upper_lim)
        ax.set_title('Force Control Tracking Performance',fontsize = 30)
        ax.legend(fontsize = 20)

    except Exception as e:
        print(f"Error updating plot: {e}")


fig, ax = plt.subplots()


animation = FuncAnimation(fig, update_plot, interval=100)


plt.show()