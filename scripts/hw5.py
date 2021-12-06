import signal
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import os.path 

# gravity constant
g = 9.8

fig, ax = plt.subplots(nrows=2, ncols=4, figsize=(24, 12), dpi=80)
# the circle plot
ax[0][0].set_xlim([-150, 150])
ax[0][0].set_ylim([530, 830])
ax[0][0].set_xlabel("X axis")
ax[0][0].set_ylabel("Z axis")

# torques
for i in range(0, 2):
    for j in range(0, 4): 
        if i==0 and j==0: 
            continue 
        ax[i][j].set_xlim([0, 400])
        ax[i][j].set_ylim([-30, 30])
        ax[i][j].set_xlabel("Seconds")
        ax[i][j].set_ylabel("Torques")
        ax[i][j].text(160, 25, "joint {}".format(j+4*i), fontsize=20)

# assume joint 3 is fixed
ax[0][3].text(130, 0, "fixed joint 3", fontsize=20,
                bbox=dict(facecolor='red', alpha=0.5))

fig.text(0.38, 0.92, "Draw circles and show the torques (200 sec/circle)", fontsize=20, 
        bbox=dict(facecolor='green', alpha=0.5))

class Kuka_7dof: 
    # link length
    d1 = 360 # link1
    d3 = 420 # link2 + link3
    d5 = 201 + 198.5 # link4 + link5
    d7 = 105.5 + 100 # link6 + pen

    m1 = 4 # mass center at joint 2
    m2 = 4.5 # mass center at joint 2
    m3 = 2.45 # mass center at joint 4
    m4 = 2.6 # mass center at joint 4
    m5 = 3.4 # mass center at joint 6
    m6 = 3.4 # mass center at joint 6

    def __init__(self):
         # initial config
        self.joints_config = [math.pi/2, 0.0002, 0, -math.pi/2, 0, 0.001, 0] # small joint value to avoid singularity
        self.T = self.forward_kinematic(self.joints_config)
        self.pos = self.T[:,3][:3] # end point origin x y z, numpy.array([[0], [605], [780]])  

        self.way_points = self.pos # paths
        self.torques_traj = np.array([[0, 0, 0, 0, 0, 0, 0]]).transpose() # initial torques
          
    def forward_kinematic(self, joints_config, link=7): 
        if(link == 0):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
        t1, t2, t3, t4, t5, t6, t7 = joints_config
    
        # DH table construction
        def Ttheta(t):
            return np.matrix([[math.cos(t), -math.sin(t), 0, 0], [math.sin(t), math.cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def Td(d):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
        
        def Talfa(alfa):
            return np.matrix([[1, 0, 0, 0], [0, math.cos(alfa), -math.sin(alfa), 0], [0, math.sin(alfa), math.cos(alfa), 0], [0, 0, 0, 1]])
        
        T1 = Ttheta(t1) * Td(Kuka_7dof.d1) * Talfa(-math.pi/2)
        if(link == 1):
            return T1
    
        T2 = Ttheta(t2) * Talfa(math.pi/2)
        if(link == 2):
            return T1*T2
    
        T3 = Ttheta(t3) * Td(Kuka_7dof.d3) * Talfa(math.pi/2)
        if(link == 3):
            return T1*T2*T3
    
        T4 = Ttheta(t4) * Talfa(-math.pi/2)
        if(link == 4):
            return T1*T2*T3*T4
    
        T5 = Ttheta(t5) * Td(Kuka_7dof.d5) * Talfa(-math.pi/2)
        if(link == 5):
            return T1*T2*T3*T4*T5
    
        T6 = Ttheta(t6) * Talfa(math.pi/2)
        if(link == 6):
            return T1*T2*T3*T4*T5*T6
    
        T7 = Ttheta(t7) * Td(Kuka_7dof.d7)
        T = T1 * T2 * T3 * T4 * T5 * T6 * T7
        return T
   
    def cal_jacobian(self, joints, end_frame=7):
        if(end_frame==7):
            #default use full T
            T = self.forward_kinematic(joints); 
        else:
            T = self.forward_kinematic(joints, link=end_frame); 

        on = T[:,3][:3] # end point origin
        g_vector = np.vstack((np.array([[0, 0]]).transpose(), on[2], np.array([[0, 0, 0]]).transpose()))   
        # mm to m
        g_vector = g_vector/1000

        def cal_jacobian_column(link): 
            T_matrix = self.forward_kinematic(joints, link)
            o = T_matrix[:,3][:3] # origin
            z = T_matrix[:,2][:3] # z axis
            j_linear = np.cross(z, on-o, axisa=0, axisb=0, axisc=0)
            j_angular = z
            j = np.vstack([j_linear, j_angular])
            if(link==7):
                # we want the end z-axis for end force analysis
                return j, z
            else:
                return j
    
        j1 = cal_jacobian_column(0) 
        j2 = cal_jacobian_column(1) 
        if(end_frame==2):
            j = np.hstack((j1, j2))
            return j, g_vector

        # set joint3 as constant to make jacobian a square matrix
        j4 = cal_jacobian_column(3) 
        if(end_frame==4):
            j = np.hstack((j1, j2, j4))
            return j, g_vector

        j5 = cal_jacobian_column(4) 
        j6 = cal_jacobian_column(5) 
        if(end_frame==6):
            j = np.hstack((j1, j2, j4, j5, j6))
            return j, g_vector


        j7, z = cal_jacobian_column(7) 
        if(end_frame==7):
            j = np.hstack((j1, j2, j4, j5, j6, j7))
            # The force is opposite to the end z axis
            # mm to m
            # append end_torques = 0
            #f_vector = np.vstack((-z, np.array([[0, 0, 0]]).transpose()))
            f_vector = np.vstack((np.array([[0, -1, 0]]).transpose(), np.array([[0, 0, 0]]).transpose()))
            return j, f_vector

    def cal_torques(self, joints, end_force):

        def force_to_torques(frame, force):
            j, vector = self.cal_jacobian(joints, end_frame=frame)
            f_vector = force * vector
            return np.matmul(j.transpose()/1000, f_vector)

        # mass
        t_m1m2 = force_to_torques(2, (Kuka_7dof.m1 + Kuka_7dof.m2) * g) 
        # m1 m2 doesn't affect joint 4,5,6,7
        t_4567 = np.array([[0, 0, 0, 0]]).transpose()
        t_m1m2 = np.vstack((t_m1m2, t_4567))

        t_m3m4 = force_to_torques(4, (Kuka_7dof.m3 + Kuka_7dof.m4) * g) 
        # m3 m4 doesn't affect joint 5,6,7
        t_567 = np.array([[0, 0, 0]]).transpose()
        t_m3m4 = np.vstack((t_m3m4, t_567))

        t_m5m6 = force_to_torques(6, (Kuka_7dof.m5 + Kuka_7dof.m6) * g) 
        # m5 m6 doesn't affect joint 7
        t_7 = np.array([[0]]).transpose()
        t_m5m6 = np.vstack((t_m5m6, t_7))

        # end force
        t_f = force_to_torques(7, end_force) 

        torques = t_m1m2 + t_m3m4 + t_m5m6 + t_f
        return torques

    def test_cal_torques(self):
        test_config = [math.pi/2, 0.0002, 0, -math.pi/2, 0, 0.001, 0]
        t = self.cal_torques(test_config, end_force=5)
        print(t)
                  
    
    def inverse_velocity(self, joints, velocity):
        j, _ = self.cal_jacobian(joints) # jacobian
        inv_j = j.getI() # inverse jacobian
        joints_velocity = inv_j * velocity 
        return np.insert(joints_velocity, 2, 0) # add back joint3 as 0
   
    def clip_angle(self, joints):
        clipped_joints = []
        for joint in joints:
            joint = joint % (2*math.pi)
            clipped_joints.append(joint)
        return clipped_joints
    
    def draw_circle(self, center=(0, 605, 680), radius=100, simulated_points=200):
        cx, cy, cz = center

        # convert cartesian cooridnate to polar coordinate
        def cart_to_polar(xyz):
            x, y, z = xyz
            r = math.sqrt(math.pow(x, 2) + math.pow(z, 2))
            print("x: ", x,", z: ", z)
            #plt.scatter(x, z)
            #plt.pause(0.0001)
            theta = math.atan2(z-cz, x-cx) # center is at z=680
            return (r, y, theta)


        # polar coordinate
        _, _, theta = cart_to_polar(self.pos.T.tolist()[0])
        polar_pos = (radius, cy, theta)
        r, y, theta = polar_pos

        time_per_circle = 200 # draw the circle in 200s
        # initial velocity
        w = (2*math.pi)/time_per_circle # angular velocity for drawing a circle in 5s
        v = r*w # linear velocity
        v_vector_T= np.array([[v*-math.sin(theta), 0, v*math.cos(theta), 0, 0, 0]]) # end point (vx, vy, vz, ax, ay, az) related to base
        v_vector = np.transpose(v_vector_T)
 
        q = self.joints_config # small joint value to avoid singularity
       
        delta_t = time_per_circle/simulated_points
        count = 0
        sim_time = 0
        while(True): 
            # calculate torques
            torques = self.cal_torques(q, end_force=5)
            torques_and_time = np.vstack((torques, sim_time))
            self.torques_traj = np.hstack((self.torques_traj, torques_and_time))

            # calculate joints velocity
            q_dot = self.inverse_velocity(q, v_vector)
            q = np.array(q) + q_dot * delta_t 
            sim_time = sim_time + delta_t
            q = self.clip_angle(q.tolist()[0])
            self.joints_config = q

            T = self.forward_kinematic(q); 
            self.pos = T[:,3][:3] # origin
            self.way_points = np.hstack((self.way_points, self.pos))
            #print("pos: ", pos)
            _, _, theta = cart_to_polar(self.pos.T.tolist()[0])
            theta = theta + 0.08*50/simulated_points # compensate the overshoot distance caused by running tangent speed in delta_t time
            v_vector_T= np.array([[v*-math.sin(theta), 0, v*math.cos(theta), 0, 0, 0]]) # end point (vx, vy, vz, ax, ay, az) related to base
            v_vector = np.transpose(v_vector_T)
            count+=1
    
            if(sim_time > time_per_circle*2):
                print("finish")
                return count

    def plot_path(self, i):
 
        x = self.way_points[0].tolist()[0][i]
        z = self.way_points[2].tolist()[0][i]
        joint1_torque = self.torques_traj[0].tolist()[0][i]
        joint2_torque = self.torques_traj[1].tolist()[0][i]
        joint4_torque = self.torques_traj[2].tolist()[0][i]
        joint5_torque = self.torques_traj[3].tolist()[0][i]
        joint6_torque = self.torques_traj[4].tolist()[0][i]
        joint7_torque = self.torques_traj[5].tolist()[0][i]
        sim_time = self.torques_traj[6].tolist()[0][i]
        ax[0][0].scatter(x, z)
        if(not i==0):
            ax[0][1].scatter(sim_time, joint1_torque, marker='o', color='black', s=1)
            ax[0][2].scatter(sim_time, joint2_torque, marker='o', color='black', s=1)
            ax[1][0].scatter(sim_time, joint4_torque, marker='o', color='black', s=1)
            ax[1][1].scatter(sim_time, joint5_torque, marker='o', color='black', s=1)
            ax[1][2].scatter(sim_time, joint6_torque, marker='o', color='black', s=1)
            ax[1][3].scatter(sim_time, joint7_torque, marker='o', color='black', s=1)


    #============================== testing function=================================
    def _fk_test(self):
        # joints config    
        config_1 = [0, 0, 0, 0, 0, 0, 0] # z = d1+d3+d5+d7
        config_2 = [math.pi/2, 0, 0, 0, 0, 0, 0] # R = [yo, -xo, zo]
        config_3 = [0, 0, 0, math.pi/2, 0, 0, 0] # R = [zo, yo, -xo], O=[-(d5+d7), 0, d1+d3]
        config_4 = [0, 0, 0, math.pi/2, 0, math.pi/2, 0] # O = [-d5, 0, d1+d3+d7]
        config_5 = [0, 0, math.pi/2, math.pi/2, 0, math.pi/2, 0] # R = [yo, -xo, zo], O=[0, -d5, d1+d3+d7]
    
        print(self.forward_kinematic(config_1))
        print(self.forward_kinematic(config_2))
        print(self.forward_kinematic(config_3))
        print(self.forward_kinematic(config_4))
        print(self.forward_kinematic(config_5))

    # testing function
    def _test_cal_jacobian(self):
        test_config = [math.pi/2, 0.0002, 0, -math.pi/2, 0, 0.001, 0]
        print("fk: ", self.forward_kinematic(test_config)[:,3][:3])
        j_test = cal_jacobian(test_config)
        print(j_test)
        print(j_test.getI()) # inverse

    # testing function
    def _test_inverse_velocity(self):
        test_config = [math.pi/2, 0.0002, 0, -math.pi/2, 0, 0.001, 0]
        polar_pos = (100, 605, 3.14159265/2) # (r, y, theta)
        r, y, theta = polar_pos
        w = (2*3.14159265)/5 # angular velocity for drawing the circle
        v = r*w # linear velocity
        v_vector_T= np.array([[v*-math.sin(theta), 0, v*math.cos(theta), 0, 0, 0]]) # end point (vx, vy, vz, ax, ay, az) related to base
        v_vector = np.transpose(v_vector_T)
        print("vel: ", v_vector)
        dq = self.inverse_velocity(test_config, v_vector)
        print("joint_vel: ", dq)
        q = np.array(test_config) + dq*0.01
        print("new joint: ", q.tolist())
        new_pos = self.forward_kinematic(q.tolist()[0])[:,3][:3]
        print("new pos: ", new_pos)
    #=================================================================================

def main():
    my_kuka = Kuka_7dof()

    points = my_kuka.draw_circle()

    anim = animation.FuncAnimation(fig, my_kuka.plot_path, frames=points, interval=100, repeat=False, save_count=points)

    #plt.show()

    #path_to_script = os.path.realpath(__file__)
    #parent_directory = os.path.dirname(path_to_script)
    #gif_file_path = os.path.join(parent_directory,'draw_circles.gif')
    #file_exists = os.path.exists(gif_file_path)
    #if(not file_exists):
    #    print("saving gif... (takes about 1 minutes)")
    writergif = animation.PillowWriter(fps=30)
    anim.save('draw_circles.gif', writer=writergif)

def test():
    my_kuka = Kuka_7dof()
    my_kuka.test_cal_torques()

def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    main()
    #test()
