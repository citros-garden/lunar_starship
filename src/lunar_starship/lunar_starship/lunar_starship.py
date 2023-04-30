import rclpy
from rclpy.node import Node
import time
import numpy as np

from . import lopt_starship as lopt

from std_msgs.msg import Float64MultiArray

class lunar_starship(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/lunar_starship/state', 10)

        #Defining states
        #Initial state vector
        self.declare_parameter('h_0', 0.0) # Initial altitude, m
        self.declare_parameter('lat_0', 0.0) # Initial latitude, deg
        self.declare_parameter('long_0', 0.0) # Initial longitude, deg
        self.declare_parameter('vn_0', 0.0) # Initial 'north' velocity, m/s
        self.declare_parameter('ve_0', 0.0) # Initial 'east' velocity, m/s
        self.declare_parameter('vd_0', 0.0) # Initial 'down' velocity, m/s
        self.declare_parameter('m_fuel_0', 3543000.0)  # Initial fuel mass, kg

        #Target state vector
        self.declare_parameter('h_f', 15000.0) # target altitude, m
        self.declare_parameter('lat_f', 3.0) # target latitude, deg
        self.declare_parameter('long_f', 0.0) # target longitude, deg
        self.declare_parameter('vn_f', 1700.0) # target 'north' velocity, m/s
        self.declare_parameter('ve_f', 0.0) # target 'east' velocity, m/s
        self.declare_parameter('vd_f', 0.0) # target 'down' velocity, m/s
        self.declare_parameter('m_fuel_f', 1000.0)  # target fuel mass, kg
        
        #Additional params
        self.declare_parameter('dry_mass', 85000.0 ) # Dry mass, kg
        self.declare_parameter('Fthrustmax', 20295000.0) # Maximum engines thrust, N
        self.declare_parameter('Isp', 308.0) # Impulse, sec

        self.declare_parameter('simulation_step', 1.0) # Simulation step duration

        self.declare_parameter('publish_freq', 1.0)   

        self.h_0 = self.get_parameter('h_0').get_parameter_value().double_value
        self.lat_0 = self.get_parameter('lat_0').get_parameter_value().double_value
        self.long_0 = self.get_parameter('long_0').get_parameter_value().double_value

        self.vn_0 = self.get_parameter('vn_0').get_parameter_value().double_value
        self.ve_0 = self.get_parameter('ve_0').get_parameter_value().double_value
        self.vd_0 = self.get_parameter('vd_0').get_parameter_value().double_value
        self.m_fuel_0 = self.get_parameter('m_fuel_0').get_parameter_value().double_value
        
        self.h_f = self.get_parameter('h_f').get_parameter_value().double_value
        self.lat_f = self.get_parameter('lat_f').get_parameter_value().double_value
        self.long_f = self.get_parameter('long_f').get_parameter_value().double_value

        self.vn_f = self.get_parameter('vn_f').get_parameter_value().double_value
        self.ve_f = self.get_parameter('ve_f').get_parameter_value().double_value
        self.vd_f = self.get_parameter('vd_f').get_parameter_value().double_value
        self.m_fuel_f = self.get_parameter('m_fuel_f').get_parameter_value().double_value

        self.dry_mass = self.get_parameter('dry_mass').get_parameter_value().double_value
        self.Fthrustmax = self.get_parameter('Fthrustmax').get_parameter_value().double_value
        self.Isp = self.get_parameter('Isp').get_parameter_value().double_value

        # Setting Moon parameters
        self.MoonRadius = 1737.1*1000
        self.Mu_Moon =  6.67*10**(-11) * 7.342e22
        self.g0 = 9.81

        self.simulation_step = self.get_parameter('simulation_step').get_parameter_value().double_value

        # Defining encounter for publisher
        self.i = 0
        # Creating target state vector
        target = [self.h_f, self.lat_f, self.long_f, self.vn_f, self.ve_f, self.vd_f, self.m_fuel_f]

        # Calculating maximum control value for the control bounds
        u_max = self.Fthrustmax / self.dry_mass
        
        # Setting bounds and guesses
        #            [ x[0],         x[1],      x[2],       x[3],       x[4],       x[5],       x[6]  ]
        #            [   h,           lat,      long,        Vn,         Ve,         Vd,        m_dot ]
        tf0 = 1000 # guess tf 
        xf0 = [self.h_f, self.lat_f, self.long_f, self.vn_f,  self.ve_f,  self.vd_f,  self.m_fuel_f] # target conditions
        x00 = [self.h_0, self.lat_0, self.long_0, self.vn_0,  self.ve_0,  self.vd_0,  self.m_fuel_0] #starting conditions
        lbx = [  0,      -90,  -90,   -10000,  -10000,  -10000,      0]
        ubx = [50000,    +90,  +90,    10000,   10000,   10000,    self.m_fuel_0]
        lbu = [ -u_max,  -u_max,  -u_max] #fn, fe, fd
        ubu = [  u_max,   u_max,   u_max] #fn, fe, fd
        btf = [10, 10000] # tf_min tf_max

        # It's necessary to define some functions for OCP. 
        # They will be used in the 'lopt.solve' function call later.

        # Defining terminal cost function
        def terminal_cost0(xf,tf,x0,t0):
                return tf
        
        # Defining path constraints function
        def path_constraints0(x,u,t):
                return [(u[0]**2 + u[1]**2 + u[2]**2) - (self.Fthrustmax / (x[6] + self.dry_mass))**2]
        
        # Defining terminal constraints function
        def terminal_constraints0(xf, tf, x0, t0): 
                tc = [  x0[0] - self.h_0, 
                        x0[1] - self.lat_0,
                        x0[2] - self.long_0, 
                        x0[3] - self.vn_0,
                        x0[4] - self.ve_0, 
                        x0[5] - self.vd_0, 
                        x0[6] - self.m_fuel_0, 
                        
                        xf[0] - self.h_f,
                        xf[1] - self.lat_f,
                        xf[2] - self.long_f,
                        xf[3] - self.vn_f, 
                        xf[4] - self.ve_f, 
                        xf[5] - self.vd_f,
                        xf[6] - self.m_fuel_f]
                return tc
        
        # Defining theoretical dynamic function for optimal control solver
        def dynamics0(x, u, t): 
                
                h   = x[0]
                lat = x[1]
                long = x[2]
                vn  = x[3]
                ve  = x[4]
                vd  = x[5]
                m   = x[6]
                        
                r_inv = 1/(h+ self.MoonRadius)
                g = self.Mu_Moon*(r_inv**2)
                
                h_dot = -vd
                lat_dot = vn * r_inv
                vn_dot = u[0] + vn * vd * r_inv - np.tan(lat) * ve**2 * r_inv
                ve_dot = u[1] + ve * vd * r_inv + np.tan(lat) * ve * vn * r_inv
                vd_dot = u[2] + g - (vn**2 + ve**2) * r_inv
                long_dot = ve * r_inv /(np.cos(lat))
                
                m_dot = - self.Fthrustmax/(self.g0 * self.Isp)

                return [h_dot, lat_dot, long_dot, vn_dot, ve_dot, vd_dot, m_dot]
        
        # Defining real dynamic function
        def real_dynamics0(x, t, fu_0, fu_1, fu_2):
            
            h   = x[0]
            lat = x[1]
            long = x[2]
            vn  = x[3]
            ve  = x[4]
            vd  = x[5]
            m   = x[6]
            

            r_inv = 1/(h+ self.MoonRadius)
            g = self.Mu_Moon*(r_inv**2)
            
            h_dot = -vd
            lat_dot = vn * r_inv
            vn_dot = fu_0(t) + vn * vd * r_inv - np.tan(lat) * ve**2 * r_inv
            ve_dot = fu_1(t) + ve * vd * r_inv + np.tan(lat) * ve * vn * r_inv
            vd_dot = fu_2(t) + g - (vn**2 + ve**2) * r_inv
            long_dot = ve * r_inv /(np.cos(lat))
            
            m_dot = - self.Fthrustmax/(self.g0 * self.Isp)

            return [h_dot, lat_dot, long_dot, vn_dot, ve_dot, vd_dot, m_dot]
        

        #################################
        # Calling simulation function using parameters and functions declared above
        self.res_x, self.res_u, self.res_t = lopt.solve(theo_dyn_func = dynamics0,
                                                        real_dyn_func = real_dynamics0,
                                                        term_cost = terminal_cost0,
                                                        path_constr = path_constraints0,
                                                        term_constr = terminal_constraints0,
                                                        ocp_tf0 = tf0,
                                                        ocp_btf = btf,
                                                        ocp_lbu = lbu,
                                                        ocp_lbx = lbx,
                                                        ocp_ubu = ubu,
                                                        ocp_ubx = ubx,
                                                        ocp_x00 = x00,
                                                        ocp_xf0 = xf0,
                                                        target = target,
                                                        simulation_step = self.simulation_step)
        
        self.state_msg = Float64MultiArray()
        timer_period = self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        self.state_msg.data = [self.res_x[0][self.i], self.res_x[1][self.i], self.res_x[2][self.i], 
                               self.res_x[3][self.i], self.res_x[4][self.i], self.res_x[5][self.i],
                               self.res_x[6][self.i]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_t):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = lunar_starship()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()