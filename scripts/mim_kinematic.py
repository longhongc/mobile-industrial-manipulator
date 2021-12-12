import signal
import sys
import numpy as np
from math import pi, sin, cos, atan2, asin, acos, sqrt

# gravity constant
g = 9.8

class MIM_Arm: 

    l1 = 0.0867
    l2 = 0.6127
    l3 = 0.5722
    l4 = 0.1157
    k1 = 0.1639
    k2 = 0.0912

    def __init__(self, init_config):
         # initial config
        self.arm_base_link_pose = [-0.2, 0, 0.321]

        self.joints_config = init_config 
        self.T = self.forward_kinematic(self.joints_config)
        self.pos = self.T[:,3][:3]  

        self.joints_traj = []
        self.way_points = self.pos 

    def forward_kinematic(self, joints_config, link=6): 
        if(link == 0):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
        t1, t2, t3, t4, t5, t6 = joints_config
    
        # DH table construction
        def Ttheta(t):
            return np.matrix([[cos(t), -sin(t), 0, 0], [sin(t), cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def Td(d):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

        def Ta(a):
            return np.matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def Talfa(alfa):
            return np.matrix([[1, 0, 0, 0], [0, cos(alfa), -sin(alfa), 0], [0, sin(alfa), cos(alfa), 0], [0, 0, 0, 1]])

        # arm_base to origin
        x, y, z = self.arm_base_link_pose
        Tbase = np.matrix([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

        T0 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        if(link == 0):
            return Tbase*T0
        
        T1 = Ttheta(t1 - pi/2) * Td(MIM_Arm.l1) * Talfa(-pi/2)
        if(link == 1):
            return Tbase*T0*T1
    
        T2 = Ttheta(t2 - pi/2) * Ta(MIM_Arm.l2)
        if(link == 2):
            return Tbase*T0*T1*T2
    
        T3 = Ttheta(t3)*Ta(MIM_Arm.l3)
        if(link == 3):
            return Tbase*T0*T1*T2*T3
    
        T4 = Ttheta(t4 + pi/2) * Td(MIM_Arm.k1) * Talfa(pi/2)
        if(link == 4):
            return Tbase*T0*T1*T2*T3*T4
    
        T5 = Ttheta(t5)*Td(MIM_Arm.l4) * Talfa(-pi/2)
        if(link == 5):
            return Tbase*T0*T1*T2*T3*T4*T5
    
        T6 = Ttheta(t6) * Td(MIM_Arm.k2)
        T = Tbase * T0 * T1 * T2 * T3 * T4 * T5 * T6
        return T

    def cal_jacobian(self, joints):
        T = self.forward_kinematic(joints); 

        on = T[:,3][:3] # end point origin

        def cal_jacobian_column(link): 
            T_matrix = self.forward_kinematic(joints, link)
            o = T_matrix[:,3][:3] # origin
            z = T_matrix[:,2][:3] # z axis
            j_linear = np.cross(z, on-o, axisa=0, axisb=0, axisc=0)
            j_angular = z
            j = np.vstack([j_linear, j_angular])

            return j
    
        j1 = cal_jacobian_column(0) 
        j2 = cal_jacobian_column(1) 
        j3 = cal_jacobian_column(2) 
        j4 = cal_jacobian_column(3) 
        j5 = cal_jacobian_column(4) 
        j6 = cal_jacobian_column(5) 

        j = np.hstack((j1, j2, j3, j4, j5, j6))
        return j

    def inverse_velocity(self, joints, velocity):
        j = self.cal_jacobian(joints) # jacobian
        inv_j = j.getI() # inverse jacobian
        joints_velocity = inv_j * velocity 
        return joints_velocity

    def clip_angle(self, joints):
        clipped_joints = []
        for joint in joints:
            joint = joint % (2 * pi)
            if(joint > pi):
                joint = -(2 * pi - joint)
            clipped_joints.append(joint)
        return clipped_joints
 
    def grasp_motion(self, init_joints, length, velocity, simulated_points):
        self.joints_traj = []
        self.joints_config = init_joints
        q = self.joints_config
        v = velocity
        v_vector_T= np.array([[0, 0, v, 0, 0, 0]]) # end point (vx, vy, vz, ax, ay, az) related to base
        v_vector = np.transpose(v_vector_T)
        motion_length = length
        total_time = motion_length / abs(v)
         

        delta_t = total_time / simulated_points
        print(delta_t)
        sim_time = 0

        count = 0
        while(True): 
            # calculate joints velocity
            q_dot = self.inverse_velocity(q, v_vector)
            q = np.array([q]).T + q_dot * delta_t 
            sim_time = sim_time + delta_t
            q = self.clip_angle(q.T.tolist()[0])
            self.joints_traj.append(q)
            self.joints_config = q

            T = self.forward_kinematic(q); 
            self.pos = T[:,3][:3] # origin
            self.way_points = np.hstack((self.way_points, self.pos))
            print("pos: ", self.pos.T.tolist()[0])
    
            count+=1
            if(sim_time > total_time):
                print("finish")
                return self.joints_traj

   
    def _fk_test(self):
        # joints config    
        config_1 = [0, 0, 0, 0, 0, 0]  
        config_2 = [pi/2, 0, 0, 0, 0, 0] 
        config_3 = [0, 0, 0, pi/2, 0, 0] 
        config_4 = [0, 0, pi/2, 0, 0, 0] 
        config_5 = [0, 0, pi/2, -pi/2, pi/2, 0] 
        diffdrive_pick_config = [1.4, 0.4, 0.8, 0.15, -1.57, 0]
    
        print("config_1: ", config_1)
        print(self.forward_kinematic(config_1))
        print("config_2: ", config_2)
        print(self.forward_kinematic(config_2))
        print("config_3: ", config_3)
        print(self.forward_kinematic(config_3))
        print("config_4: ", config_4)
        print(self.forward_kinematic(config_4))
        print("config_5: ", config_5)
        print(self.forward_kinematic(config_5))
        print("config_6: ", diffdrive_pick_config)
        print(self.forward_kinematic(diffdrive_pick_config))

    def _iv_test(self):
        joints_config = [1.4, 0.4, 0.8, 0.15, -1.57, 0]
        v = 0.3
        v_vector_T= np.array([[0, 0, -v, 0, 0, 0]]) # end point (vx, vy, vz, ax, ay, az) related to base
        v_vector = np.transpose(v_vector_T)
        print(v_vector)
        joints_vel = self.inverse_velocity(joints_config, v_vector)
        print(joints_vel)
        


def main():
    init_config = [1.4, 0.42, 0.8, 0.15, -1.57, 0]
    mim_arm = MIM_Arm(init_config)
    #mim_arm._fk_test()
    #mim_arm._iv_test()
    joints_traj = mim_arm.grasp_motion()
    print(joints_traj)

def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    main()
