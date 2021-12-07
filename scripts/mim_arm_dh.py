import signal
import sys
import numpy as np
import math

# gravity constant
g = 9.8

class MIM_Arm: 

    d1 = 0.0867
    a1 = 0.176
    a2 = -0.6121
    d2 = -0.12781
    a3 = -0.5722
    d3 = 0.1157
    d4 = 0.1157
    a4 = 0.0912

    def __init__(self):
         # initial config
        self.joints_config = [0, 0, 0, 0, 0] # small joint value to avoid singularity

    def forward_kinematic(self, joints_config, link=6): 
        if(link == 0):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
        t1, t2, t3, t4, t5 = joints_config
    
        # DH table construction
        def Ttheta(t):
            return np.matrix([[math.cos(t), -math.sin(t), 0, 0], [math.sin(t), math.cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def Td(d):
            return np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

        def Ta(a):
            return np.matrix([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def Talfa(alfa):
            return np.matrix([[1, 0, 0, 0], [0, math.cos(alfa), -math.sin(alfa), 0], [0, math.sin(alfa), math.cos(alfa), 0], [0, 0, 0, 1]])
        
        T1 = Ttheta(t1) * Td(MIM_Arm.d1)
        if(link == 1):
            return T1
    
        T2 = Ta(MIM_Arm.a1) * Ttheta(math.pi/2) * Talfa(math.pi/2)
        if(link == 2):
            return T1*T2
    
        T3 = Ttheta(t2-math.pi/2) * Ta(MIM_Arm.a2)
        if(link == 3):
            return T1*T2*T3
    
        T4 = Ttheta(t3) * Td(MIM_Arm.d2) * Ta(MIM_Arm.a3)
        if(link == 4):
            return T1*T2*T3*T4
    
        T5 = Ttheta(t4+math.pi/2) * Td(MIM_Arm.d3) * Talfa(-math.pi/2) * Ttheta(-math.pi/2) * Td(MIM_Arm.d4)
        if(link == 5):
            return T1*T2*T3*T4*T5
    
        Tt = Ttheta(t5) * Ta(MIM_Arm.a4)
        T = T1 * T2 * T3 * T4 * T5 * Tt
        return T
   
    def _fk_test(self):
        # joints config    
        config_1 = [0, 0, 0, 0, 0]  
        config_2 = [math.pi/2, 0, 0, 0, 0] 
        config_3 = [0, 0, 0, math.pi/2, 0] 
        config_4 = [0, 0, math.pi/2, 0, 0] 
        config_5 = [0, 0, math.pi/2, -math.pi/2, math.pi/2] 
        diffdrive_pick_config = [1.42, 0.3, 0.75, 0.15, -1.52]
    
        print("== test1 ==")
        print(self.forward_kinematic(config_1))
        print("== test2 ==")
        print(self.forward_kinematic(config_2))
        print("== test3 ==")
        print(self.forward_kinematic(config_3))
        print("== test4 ==")
        print(self.forward_kinematic(config_4))
        print("== test5 ==")
        print(self.forward_kinematic(config_5))
        print("== test6 ==")
        print(self.forward_kinematic(diffdrive_pick_config))


def main():
    mim_arm = MIM_Arm()
    mim_arm._fk_test()

def signal_handler(sig, frame):
    print("")
    print("bye bye!")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    main()
