import math
import numpy as np
from Cluster import Cluster, ClusterConfig
import sympy as sp 


FREQ = 10
JOY_FREQ = FREQ
KP_GAIN = 1.0
KV_GAIN = 1.0
EPSILON = 1.0
MAX_VEL = 0.5
ROVER_DOF = 3 # (x, y, theta)

class ClusterTestbench():
    def __init__(self):
        self.robot_id_list = ["p1", "p2", "p3", "p4", "p5"]
        self.n_rover = len(self.robot_id_list)

        #All data on actual robots
        self.cluster = None #cluster object to be initialized when cluster is formed
        self.cluster_size = self.n_rover
        self.cluster_robots = self.robot_id_list #list of active robots in cluster
        self.registered_robots = [] #list of active robots in cluster
        self.r = None #stores latest robot state space variables
        self.rd = None #stores latest robot velocities 
        self.time = None
        self.cluster = Cluster(numRobots = self.cluster_size, cluster_type=ClusterConfig.TRILEAD, cluster_params=[3,3,3,3,0,0,0], KPgains=[KP_GAIN]*(self.cluster_size*ROVER_DOF), KVgains=[KV_GAIN]*(self.cluster_size*ROVER_DOF))
        self.assign_robots()

    #after listening for nearby robots assign them to cluster 
    def assign_robots(self):
        #self.cluster_robots.sort() #sort the robot ids
        #actual robots
        self.r = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.rd = np.zeros((self.cluster_size*ROVER_DOF, 1))
    
    def set_robot_position(self, r):
        self.r = r.reshape((self.cluster_size*ROVER_DOF, 1))
    
    def set_desired_position(self, c_des):
        self.cluster.set_cdes(c_des)
    
    def check_expressions(self):
        c, r = self.cluster.test_transforms(self.r)
        print("CHECKING EXPRESSIONS")

        print(f"Cluster status/calcc: {list(c.flatten())}")
        print(f"Cluster status/calcr: {list(r.flatten())}")
        print(f"Cluster status/org r: {list(self.r.flatten())}")
   
    #Publishes velocity commands to robots in either sim or actual
    def calc_velocities(self):
        #self.get_logger().info(f"Publishing velocities to robots:{self.cluster_robots} with output: {output}")
        _r = self.r
        _rd = self.rd
        _cdes = self.cluster.c_des
        cd, rd, c_cur= self.cluster.get_velocity_command(_r , _rd)
        print(f"Cluster status/c_des: {list(_cdes.flatten())}")
        print(f"Cluster status/c_cur: {list(c_cur.flatten())}")
        print(f"Cluster status/cd: {list(cd.flatten())}")
        print(f"Cluster status/rd: {list(rd.flatten())}")
    
    def print_Jacobs(self):
        with open('cluster_expressions/FKine.txt', 'w') as f:
            f.write(sp.pretty(self.cluster.FKine, wrap_line=False))
        with open('cluster_expressions/IKine.txt', 'w') as f:
            f.write(sp.pretty(self.cluster.IKine, wrap_line=False))
        with open('cluster_expressions/Jacob.txt', 'w') as f:
            f.write(sp.pretty(self.cluster.Jacob, wrap_line=False))
        with open('cluster_expressions/JacobInv.txt', 'w') as f:
            f.write(sp.pretty(self.cluster.JacobInv, wrap_line=False))

def main(args=None):
    bench = ClusterTestbench()
    bench.print_Jacobs()

    bench.set_robot_position(np.array([2.3927992515382357e-05,-3.303924322128296,-0.0005502336425706744,5.684108734130859,6.1704912185668945,-2.7238240242004395,-2.9999594688415527,-3.3039321899414062,-0.00030073511879891157,5.578272819519043,8.915911674499512,-2.7251136302948,-3.0000298023223877,-0.30391114950180054,0.0007691276259720325]))
    bench.check_expressions()
    bench.set_robot_position(np.array([0.0,0.0,0.0, 3.0,0.0,0.0, -3.0,0.0,0.0, 3.0,3.0,0.0, -3.0, 3.0,0.0]))
    bench.check_expressions()
    bench.set_robot_position(np.array([0.0,0.0,0.0, 3.0,1.0,0.0, -3.0,-1.0,0.0, 2.0,4.0,0.0, -3.0, 3.0,0.0]))
    bench.check_expressions()





    bench.set_desired_position(np.array([0.0,-3.3941993713378906,0.0,0.0,0.0,0.0,0.0,0.0,3.0,3.0,3.0,3.0,0.0,0.0,0.0]))
    
    bench.calc_velocities()



if __name__ == '__main__':
    main()