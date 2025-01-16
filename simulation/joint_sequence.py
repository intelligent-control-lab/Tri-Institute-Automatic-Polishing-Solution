import rospy
from std_msgs.msg import Float64
import numpy as np
import os

rospy.init_node('joint_angle_publisher', anonymous=True)


folder_path = "../joint_sequence_sim/"
output_file_path = "../joint_sequence_sim/joint_sim.txt"


def read_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    return content


def read_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read().strip()
    return content

def get_files_with_extension(folder_path, extension):
    files = [f for f in os.listdir(folder_path) if f.endswith(f"{extension}")]
    return files




joint_controllers = [
    '/gp50_gazebo/joint1_position_controller/command',
    '/gp50_gazebo/joint2_position_controller/command',
    '/gp50_gazebo/joint3_position_controller/command',
    '/gp50_gazebo/joint4_position_controller/command',
    '/gp50_gazebo/joint5_position_controller/command',
    '/gp50_gazebo/joint6_position_controller/command',
]









def publish_joint_angle(joint_controller, angle):
    pub = rospy.Publisher(joint_controller, Float64, queue_size=10)
    rospy.sleep(0.0005)
    pub.publish(angle)


def interpolate(start, end, alpha):
    return start + alpha * (end - start)








            





















def combine_files(output_file_path, max_files=100):
    with open(output_file_path, 'w') as output_file:
        for i in range(0, max_files + 1):
            input_file_path1 = "../joint_sequence_sim/"+ "move2start_traj" + f"{i}.txt"
            try:
                with open(input_file_path1, 'r') as input_file:
                    content = input_file.read().strip()
                    output_file.write(content + '\n')
            except FileNotFoundError:
                break
            input_file_path2 = "../joint_sequence_sim/"+ "traj" + f"{i}.txt"
            try:
                with open(input_file_path2, 'r') as input_file:
                    content = input_file.read().strip()
                    output_file.write(content + '\n')
            except FileNotFoundError:
                break
            

    print(f"Combined files into: {output_file_path}")

if __name__ == '__main__':




    print(f'Merged files into: {output_file_path}')
    joint_txt_file = "../joint_sequence_sim/joint_sim.txt"
    

    try:

        with open(joint_txt_file, 'r') as file:
            lines = file.readlines()

        for i in range(len(lines) - 1):
            start_angles = [float(angle) * np.pi / 180 for angle in lines[i].split()]
            end_angles = [float(angle) * np.pi / 180 for angle in lines[i + 1].split()]

            steps = 10
            for step in range(steps + 1):
                alpha = step / steps
                interpolated_angles = [interpolate(start, end, alpha) for start, end in zip(start_angles, end_angles)]

                for joint_controller, angle in zip(joint_controllers, interpolated_angles):
                    publish_joint_angle(joint_controller, angle)
                print(interpolated_angles)
                rospy.sleep(0.0005)

        print("process finished")
    except rospy.ROSInterruptException:
        pass



