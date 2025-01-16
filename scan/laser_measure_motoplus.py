"""
The brand new laser scan setup

Author: Weiye Zhao
Email: weiyezha@andrew.cmu.edu
"""
import time
import ctypes as ct
import numpy as np
import os
import threading
import pylinllt as llt
import socket
import struct
from detection_magic import TextColor
class LaserSubscriber:
    """
    This is a class for subscribing the laser scan data.
    It is used with the scanControl laser scanner.
    It subscribes to the laser topic and save to file.

    Attributes:
        saveDir (string): The folder that the laser scan data is saved to.
    """
    def __init__(self, saveDir=None):
        """
        The constructor for LaserSubscriber class.

        Parameters: 
            saveDir (string): The folder that the laser scan data is saved to.
        """
        self.rootDir = saveDir
        self.x=[]
        self.z=[]
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.joint6 = 0
        self.joints = []
        self.max_not_move_count = 30

    def listen_real_time_robot_configuration(self):
        """real time return the robot joint configuration
         
        Returns:
            ndarray: current stable joint position
        """        
        server_ip = '192.168.1.31'
        server_port = 11002
        message = "get_pos"
        localIP = '0.0.0.0'

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.bind((localIP,server_port))
        

        not_move_count = 0
        while(1):


            data, server_address = client_socket.recvfrom(24)
            joint_pos= []
            for i in range(0,24,4):
                temp = struct.unpack('f', data[i:i+4])
                joint_pos.append(temp)
            
            joint_pos_current_frame = np.array(joint_pos)
            

            if 'joint_pos_pre_frame' not in locals():
                joint_pos_pre_frame = joint_pos_current_frame
            

            if (joint_pos_current_frame - joint_pos_pre_frame).max() < 0.00001:

                not_move_count += 1
            else:

                not_move_count = 0
                

            if not_move_count > self.max_not_move_count:
                print(f"{TextColor.BOLD}{TextColor.GREEN}robot position is stable at {joint_pos_current_frame.flatten()}.{TextColor.RESET}")
                return joint_pos_current_frame
            
    def listen(self):
        """
        The function to subscribe the laser data and save to file. 
        Based on scanControl SDK
        """
        def profile_callback(data, size, user_data):
            """
            The callback function to retrieve laser data.
            """
            user_data_cast = ct.cast(user_data, ct.POINTER(ct.c_uint))
            if user_data_cast.contents.value == 1:

                ct.memmove(profile_buffer, data, size)
                event.set()
                self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]


        start_data = 4
        data_width = 4
        scanner_type = ct.c_int(0)


        timestamp = (ct.c_ubyte * 16)()
        available_resolutions = (ct.c_uint * 4)()

        available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
        available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

        shutter_opened = ct.c_double(0.0)
        shutter_closed = ct.c_double(0.0)
        profile_count = ct.c_uint(0)
        cb_user_data = ct.c_uint(1)

        get_profile_cb = llt.buffer_cb_func(profile_callback)
        event = threading.Event()


        null_ptr_short = ct.POINTER(ct.c_ushort)()
        null_ptr_int = ct.POINTER(ct.c_uint)()

        hLLT = llt.create_llt_device()

        ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
 
        if ret < 1:
            raise ValueError("Error getting interfaces : " + str(ret))

        ret = llt.set_device_interface(hLLT, available_interfaces[0])
        if ret < 1:
            raise ValueError("Error setting device interface: " + str(ret))


        ret = llt.connect(hLLT)
        if ret < 1:
            raise ConnectionError("Error connect: " + str(ret))


        ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        resolution = available_resolutions[0]
        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        profile_buffer = (ct.c_ubyte*(resolution * data_width))()
        x = np.empty(resolution, dtype=float)
        z = np.empty(resolution, dtype=float)
        x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
        z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))


        partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)


        ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
        if ret < 1:
            raise ValueError("Error scanner type: " + str(ret))


        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error setting resolution: " + str(ret))


        ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
        if ret < 1:
            raise ValueError("Error setting profile config: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
        if ret < 1:
            raise ValueError("Error setting trigger: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, 100)
        if ret < 1:
            raise ValueError("Error setting exposure time: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, 900)
        if ret < 1:
            raise ValueError("Error idle time: " + str(ret))


        ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
        if ret < 1:
            raise ValueError("Error setting partial profile: " + str(ret))


        ret = llt.register_buffer_callback(hLLT, get_profile_cb, ct.byref(cb_user_data))
        if ret < 1:
            raise ValueError("Error setting callback: " + str(ret))


        ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError("Error starting transfer profiles: " + str(ret))


        time.sleep(1)

        def data_gen(*args):
            event.wait()
            fret = llt.convert_part_profile_2_values(profile_buffer, len(profile_buffer), ct.byref(partial_profile_struct), scanner_type, 0,
                                                null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
            if fret & llt.CONVERT_X is 0 or fret & llt.CONVERT_Z is 0:
                raise ValueError("Error converting data: " + str(ret))

            for i in range(16):
                timestamp[i] = profile_buffer[resolution * data_width - 16 + i]

            llt.timestamp_2_time_and_count(timestamp, ct.byref(shutter_opened), ct.byref(shutter_closed), ct.byref(profile_count), null_ptr_short)
            event.clear()

            return x, z

        count = 0
        joints = np.zeros(resolution)
        idx = 0
        pre_j1 = self.joints[0]
        pre_j2 = self.joints[1]
        pre_j3 = self.joints[2]
        pre_j4 = self.joints[3]
        pre_j5 = self.joints[4]
        pre_j6 = self.joints[5]
        


        server_ip = '192.168.1.31'
        server_port = 11002
        message = "get_pos"
        localIP = '0.0.0.0'

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('localhost',server_port)

        
        while(1):

            message = "get_pos"
            client_socket.sendto(message.encode(), (server_ip, server_port))


            tic = time.time()
            data, server_address = client_socket.recvfrom(24)
            joint_pos= []
            for i in range(0,24,4):
                temp = struct.unpack('f', data[i:i+4])
                joint_pos.append(temp)
            toc = time.time()
            print(f'motoplus read time is {toc - tic}')
            

            
            laser_data = []
            tic = time.time()
            x, z = data_gen() 
            toc = time.time()
            print(f'laser read time is {toc - tic}')
            joints[0] = joint_pos[0][0]
            joints[1] = joint_pos[1][0]
            joints[2] = joint_pos[2][0]
            joints[3] = joint_pos[3][0]
            joints[4] = joint_pos[4][0]
            joints[5] = joint_pos[5][0]
            
            laser_data.append(joints)
            laser_data.append(x)
            laser_data.append(z)

            if(self.robotNotMove(pre_j1, pre_j2, pre_j3, pre_j4, pre_j5, pre_j6, joints)):
                count+=1
                if(count < 3):
                    np.savetxt(self.rootDir+str(idx)+'.txt', np.asarray(laser_data))
                    idx += 1
            else:

                print(joint_pos)
                np.savetxt(self.rootDir+str(idx)+'.txt', np.asarray(laser_data))
                idx += 1
                count = 0

                pre_j1 = joints[0]
                pre_j2 = joints[1]
                pre_j3 = joints[2]
                pre_j4 = joints[3]
                pre_j5 = joints[4]
                pre_j6 = joints[5]

    def listen_fake_joint(self):
        """
        The function to subscribe the laser data and save to file. 
        Based on scanControl SDK
        """
        def profile_callback(data, size, user_data):
            """
            The callback function to retrieve laser data.
            """
            user_data_cast = ct.cast(user_data, ct.POINTER(ct.c_uint))
            if user_data_cast.contents.value == 1:

                ct.memmove(profile_buffer, data, size)
                event.set()
                self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]


        start_data = 4
        data_width = 4
        scanner_type = ct.c_int(0)


        timestamp = (ct.c_ubyte * 16)()
        available_resolutions = (ct.c_uint * 4)()

        available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
        available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

        shutter_opened = ct.c_double(0.0)
        shutter_closed = ct.c_double(0.0)
        profile_count = ct.c_uint(0)
        cb_user_data = ct.c_uint(1)

        get_profile_cb = llt.buffer_cb_func(profile_callback)
        event = threading.Event()


        null_ptr_short = ct.POINTER(ct.c_ushort)()
        null_ptr_int = ct.POINTER(ct.c_uint)()

        hLLT = llt.create_llt_device()

        ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
 
        if ret < 1:
            raise ValueError("Error getting interfaces : " + str(ret))

        ret = llt.set_device_interface(hLLT, available_interfaces[0])
        if ret < 1:
            raise ValueError("Error setting device interface: " + str(ret))


        ret = llt.connect(hLLT)
        if ret < 1:
            raise ConnectionError("Error connect: " + str(ret))


        ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        resolution = available_resolutions[0]
        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        profile_buffer = (ct.c_ubyte*(resolution * data_width))()
        x = np.empty(resolution, dtype=float)
        z = np.empty(resolution, dtype=float)
        x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
        z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))


        partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)


        ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
        if ret < 1:
            raise ValueError("Error scanner type: " + str(ret))


        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error setting resolution: " + str(ret))


        ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
        if ret < 1:
            raise ValueError("Error setting profile config: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
        if ret < 1:
            raise ValueError("Error setting trigger: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, 100)
        if ret < 1:
            raise ValueError("Error setting exposure time: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, 900)
        if ret < 1:
            raise ValueError("Error idle time: " + str(ret))


        ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
        if ret < 1:
            raise ValueError("Error setting partial profile: " + str(ret))


        ret = llt.register_buffer_callback(hLLT, get_profile_cb, ct.byref(cb_user_data))
        if ret < 1:
            raise ValueError("Error setting callback: " + str(ret))


        ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError("Error starting transfer profiles: " + str(ret))


        time.sleep(1)

        def data_gen(*args):
            event.wait()
            fret = llt.convert_part_profile_2_values(profile_buffer, len(profile_buffer), ct.byref(partial_profile_struct), scanner_type, 0,
                                                null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
            if fret & llt.CONVERT_X is 0 or fret & llt.CONVERT_Z is 0:
                raise ValueError("Error converting data: " + str(ret))

            for i in range(16):
                timestamp[i] = profile_buffer[resolution * data_width - 16 + i]

            llt.timestamp_2_time_and_count(timestamp, ct.byref(shutter_opened), ct.byref(shutter_closed), ct.byref(profile_count), null_ptr_short)
            event.clear()

            return x, z

        joints = np.zeros(resolution)
        idx = 0
        
        while(1):
            joint_pos = np.ones((6,1), dtype=np.float32)

            laser_data = []
            x, z = data_gen() 
            joints[0] = joint_pos[0][0]
            joints[1] = joint_pos[1][0]
            joints[2] = joint_pos[2][0]
            joints[3] = joint_pos[3][0]
            joints[4] = joint_pos[4][0]
            joints[5] = joint_pos[5][0]
            
            laser_data.append(joints)
            laser_data.append(x)
            laser_data.append(z)

            print(f'data index {idx} logged')
            np.savetxt(self.rootDir+str(idx)+'.txt', np.asarray(laser_data))
            idx += 1

            time.sleep(0.3)

    

    def listen_stable(self):
        """
        The function to subscribe the laser data and save to file. 
        Based on scanControl SDK
        """
        def profile_callback(data, size, user_data):
            """
            The callback function to retrieve laser data.
            """
            user_data_cast = ct.cast(user_data, ct.POINTER(ct.c_uint))
            if user_data_cast.contents.value == 1:

                ct.memmove(profile_buffer, data, size)
                event.set()
                self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]


        start_data = 4
        data_width = 4
        scanner_type = ct.c_int(0)


        timestamp = (ct.c_ubyte * 16)()
        available_resolutions = (ct.c_uint * 4)()

        available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
        available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

        shutter_opened = ct.c_double(0.0)
        shutter_closed = ct.c_double(0.0)
        profile_count = ct.c_uint(0)
        cb_user_data = ct.c_uint(1)

        get_profile_cb = llt.buffer_cb_func(profile_callback)
        event = threading.Event()


        null_ptr_short = ct.POINTER(ct.c_ushort)()
        null_ptr_int = ct.POINTER(ct.c_uint)()

        hLLT = llt.create_llt_device()

        ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
 
        if ret < 1:
            raise ValueError("Error getting interfaces : " + str(ret))

        ret = llt.set_device_interface(hLLT, available_interfaces[0])
        if ret < 1:
            raise ValueError("Error setting device interface: " + str(ret))


        ret = llt.connect(hLLT)
        if ret < 1:
            raise ConnectionError("Error connect: " + str(ret))


        ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        resolution = available_resolutions[0]
        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))


        profile_buffer = (ct.c_ubyte*(resolution * data_width))()
        x = np.empty(resolution, dtype=float)
        z = np.empty(resolution, dtype=float)
        x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
        z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))


        partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)


        ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
        if ret < 1:
            raise ValueError("Error scanner type: " + str(ret))


        ret = llt.set_resolution(hLLT, resolution)
        if ret < 1:
            raise ValueError("Error setting resolution: " + str(ret))


        ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
        if ret < 1:
            raise ValueError("Error setting profile config: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
        if ret < 1:
            raise ValueError("Error setting trigger: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, 100)
        if ret < 1:
            raise ValueError("Error setting exposure time: " + str(ret))


        ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, 900)
        if ret < 1:
            raise ValueError("Error idle time: " + str(ret))


        ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
        if ret < 1:
            raise ValueError("Error setting partial profile: " + str(ret))


        ret = llt.register_buffer_callback(hLLT, get_profile_cb, ct.byref(cb_user_data))
        if ret < 1:
            raise ValueError("Error setting callback: " + str(ret))


        ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError("Error starting transfer profiles: " + str(ret))


        time.sleep(1)

        def data_gen(*args):
            event.wait()
            fret = llt.convert_part_profile_2_values(profile_buffer, len(profile_buffer), ct.byref(partial_profile_struct), scanner_type, 0,
                                                null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
            if fret & llt.CONVERT_X is 0 or fret & llt.CONVERT_Z is 0:
                raise ValueError("Error converting data: " + str(ret))

            for i in range(16):
                timestamp[i] = profile_buffer[resolution * data_width - 16 + i]

            llt.timestamp_2_time_and_count(timestamp, ct.byref(shutter_opened), ct.byref(shutter_closed), ct.byref(profile_count), null_ptr_short)
            event.clear()

            return x, z

        count = 0
        stable_count = 0
        written = False
        joints = np.zeros(resolution)
        idx = 0
        pre_j1 = self.joints[0]
        pre_j2 = self.joints[1]
        pre_j3 = self.joints[2]
        pre_j4 = self.joints[3]
        pre_j5 = self.joints[4]
        pre_j6 = self.joints[5]
        
        server_ip = '192.168.1.31'
        server_port = 11002
        message = "get_pos"
        localIP = '0.0.0.0'

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.bind((localIP,server_port))
        server_address = ('localhost',server_port)
        while(1):
            data, server_address = client_socket.recvfrom(24)
            joint_pos= []
            for i in range(0,24,4):
                temp = struct.unpack('f', data[i:i+4])
                joint_pos.append(temp)
            
            
            laser_data = []
            x, z = data_gen() 
            joints[0] = joint_pos[0][0]
            joints[1] = joint_pos[1][0]
            joints[2] = joint_pos[2][0]
            joints[3] = joint_pos[3][0]
            joints[4] = joint_pos[4][0]
            joints[5] = joint_pos[5][0]
            
            laser_data.append(joints)
            laser_data.append(x)
            laser_data.append(z)
            if self.robotStable(pre_j1, pre_j2, pre_j3, pre_j4, pre_j5, pre_j6, joints):
                stable_count += 1
                if stable_count > 6 and not written:
                    print(joint_pos)

                    np.savetxt(self.rootDir+str(idx)+'.txt', np.asarray(laser_data))

                    idx += 1 
                    written = True    
            else: 

                stable_count = 0
                written = False
            

            pre_j1 = joints[0]
            pre_j2 = joints[1]
            pre_j3 = joints[2]
            pre_j4 = joints[3]
            pre_j5 = joints[4]
            pre_j6 = joints[5]    
            
    
    def listen_tool_cali_pos(self):
        count = 0
        stable_count = 0
        written = False
        joints = np.zeros(6)
        idx = 0
        self.joints = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]
        pre_j1 = self.joints[0]
        pre_j2 = self.joints[1]
        pre_j3 = self.joints[2]
        pre_j4 = self.joints[3]
        pre_j5 = self.joints[4]
        pre_j6 = self.joints[5]
        
        server_ip = '192.168.1.31'
        server_port = 11002
        message = "get_pos"
        localIP = '0.0.0.0'

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('localhost',server_port)
        client_socket.bind((localIP,server_port))
        start_read = False
        
        server_port = 11002
        message = "get_pos"
        localIP = '0.0.0.0'

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.bind((localIP,server_port))
        while(1):

            while not start_read:
                user_input = input("Press 'c' to break the loop: ")

                if user_input.lower() == 'c':
                    print("start read a stable pose.")
                    start_read = True
                else:
                    print("Invalid input. Try again.")


            data, server_address = client_socket.recvfrom(24)
            joint_pos= []
            for i in range(0,24,4):
                temp = struct.unpack('f', data[i:i+4])
                joint_pos.append(temp)
            
            

            
            laser_data = []
            joints[0] = joint_pos[0][0]
            joints[1] = joint_pos[1][0]
            joints[2] = joint_pos[2][0]
            joints[3] = joint_pos[3][0]
            joints[4] = joint_pos[4][0]
            joints[5] = joint_pos[5][0]
            
            laser_data.append(joints)
            

            if self.robotStable(pre_j1, pre_j2, pre_j3, pre_j4, pre_j5, pre_j6, joints):
                stable_count += 1
                if stable_count > 30 and not written:
                    print(joint_pos)
                    np.savetxt(self.rootDir+str(idx)+'.txt', np.asarray(laser_data))

                    idx += 1 
                    written = True
                    start_read = False
                    
            else: 

                stable_count = 0
                written = False
            pre_j1 = joints[0]
            pre_j2 = joints[1]
            pre_j3 = joints[2]
            pre_j4 = joints[3]
            pre_j5 = joints[4]
            pre_j6 = joints[5]  
    
    def robotStable(self, j1, j2, j3, j4, j5, j6, joints):
        if(self.equal_stable(j1, joints[0]) and self.equal_stable(j2, joints[1]) and self.equal_stable(j3, joints[2]) and 
           self.equal_stable(j4, joints[3]) and self.equal_stable(j5, joints[4]) and self.equal_stable(j6, joints[5])):
            return True
        return False
    
    def equal_stable(self, a, b, epsilon=0.0000001):
        """
        The function to check if two numbers are equal.

        Parameters: 
            a (float): an input number
            b (float): an input number
            epsilon (float): tolerance for equality. Default: 0.01
        """
        if(abs(a-b)<=epsilon):
            return True
        return False
    
    
    def robotNotMove(self, j1, j2, j3, j4, j5, j6, joints):
        if(self.equal(j1, joints[0]) and self.equal(j2, joints[1]) and self.equal(j3, joints[2]) and 
           self.equal(j4, joints[3]) and self.equal(j5, joints[4]) and self.equal(j6, joints[5])):
            return True
        return False


    def equal(self, a, b, epsilon=0.001):
        """
        The function to check if two numbers are equal.

        Parameters: 
            a (float): an input number
            b (float): an input number
            epsilon (float): tolerance for equality. Default: 0.01
        """
        if(abs(a-b)<=epsilon):
            return True
        return False
            

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        folder_path = sys.argv[1]

        folder_path = os.path.join(os.getcwd(), folder_path)
        os.makedirs(folder_path, exist_ok=True)
    else:

        folder_path = 'scan_data_temporary/'

    files = os.listdir(folder_path)
    for file in files:
        file_path = os.path.join(folder_path, file)
        if os.path.isfile(file_path):
            try:
                os.remove(file_path)

            except Exception as e:
                print(f"Error removing file {file_path}: {e}")
                    
    print("clear finished, now start listening!")
    laserSub = LaserSubscriber(folder_path)    
    laserSub.listen_stable()


        
        

