from . import mpopt_lulav as mp_l
import numpy as np
from scipy. integrate import odeint
from scipy.interpolate import interp1d
from typing import Callable

def get_variables(post, stage_res): #Getting states values from solver and interpolating 
    
    x,u,t,a = post.get_data(phases=[0], interpolate = True)
    
    t = [t[i] for i in range(len(t))]
    stage_res.t = interp1d(t, t, kind='cubic', fill_value="extrapolate")

    stage_res.h=interp1d(t, x[:,0], kind='cubic', fill_value="extrapolate")
    
    stage_res.lat=interp1d(t, x[:,1], kind='cubic', fill_value="extrapolate")
    
    stage_res.long=interp1d(t, x[:,2], kind='cubic', fill_value="extrapolate")
    
    stage_res.vn=interp1d(t, x[:,3], kind='cubic', fill_value="extrapolate")
        
    stage_res.ve = interp1d(t, x[:,4], kind='cubic', fill_value="extrapolate")

    stage_res.vd= interp1d(t, x[:,5], kind='cubic', fill_value="extrapolate")

    stage_res.m_fuel= interp1d(t, x[:,6], kind='cubic', fill_value="extrapolate")

    stage_res.u = [interp1d(t, u[:,0], kind='cubic', fill_value="extrapolate"), 
                   interp1d(t, u[:,1], kind='cubic', fill_value="extrapolate"),
                   interp1d(t, u[:,2], kind='cubic', fill_value="extrapolate")]
    
    return (stage_res)

class Stage_result(object): #Defining a class for results
    def __init__(self):
        self.t = list()
        
        self.h = list()
        
        self.lat = list()

        self.long = list()
        
        self.vn = list()
        
        self.ve = list()

        self.vd = list()

        self.m_fuel = list()

        self.u = list()
        pass
    
    def add_states(self, stage_res, moment): #Manually adding results into Stage_result object for output
        self.t.append(float(moment))
        self.h.append(float(stage_res.h(0)))
        self.lat.append(float(stage_res.lat(0)))
        self.long.append(float(stage_res.long(0)))
        self.vn.append(float(stage_res.vn(0)))
        self.ve.append(float(stage_res.ve(0)))
        self.vd.append(float(stage_res.vd(0)))
        self.m_fuel.append(float(stage_res.m_fuel(0)))
        self.u.append([float(stage_res.u[0](0)), float(stage_res.u[1](0)), float(stage_res.u[2](0))])

        return self

def solve(theo_dyn_func : Callable =  lambda x, u, t: [x[1], u[0] - 1.5],
          real_dyn_func : Callable =  lambda x, u, t: [x[1], u[0] - 1.5],
         term_cost: Callable = lambda xf, tf, x0, t0: tf,
         path_constr: Callable = lambda x, u, t: u[0],
         term_constr: Callable = lambda xf, tf, x0, t0: [],
         ocp_tf0: int = 10, ocp_x00: list = [], ocp_xf0: list = [], ocp_lbx: list = [], 
         ocp_ubx: list = [], ocp_lbu: list = [], ocp_ubu: list = [], ocp_btf: list = [],
         target: list = [0,0,0,0,0,0,0], simulation_step: float = 1):

    ocp = mp_l.OCP(n_states=7, n_controls=3)

    if ocp_tf0:
        ocp.tf0[0] = ocp_tf0 # guess tf
    if ocp_xf0:
        ocp.xf0[0] = ocp_xf0 # target conditions
    if ocp_x00:
        ocp.x00[0] = ocp_x00 #starting conditions
    if ocp_lbx:    
        ocp.lbx[0] = ocp_lbx
    if ocp_ubx:    
        ocp.ubx[0] = ocp_ubx
    if ocp_lbu:
        ocp.lbu[0] = ocp_lbu #u_x, u_y, u_z
    if ocp_ubu:
        ocp.ubu[0] = ocp_ubu #u_x, u_y, u_z
    if ocp_btf:
        ocp.lbtf[0], ocp.ubtf[0] = ocp_btf[0], ocp_btf[1] # tf_min tf_max

    #Setting parameters
    G = 6.67*10**(-11) #real-world value is : G = 6.67e-11 

    #Moon settings
    MoonMass=7.342e22 #kg #real-world value is 7.342*10**(22) kg
    MoonRadius = 1737.1*1000
    Mu_Moon=G*MoonMass

    def init_real_flight(theo_stage_res, prev_real_stage_res, step): #Initializing stage with another dynamics

        t_array=[ prev_real_stage_res.t.y[i] for i in range(len(prev_real_stage_res.t.y))] #Preparing time array

        # Interpolating controls
        fu_0 = interp1d(t_array, theo_stage_res.u[0].y, kind='cubic', fill_value="extrapolate") 
        fu_1 = interp1d(t_array, theo_stage_res.u[1].y, kind='cubic', fill_value="extrapolate")
        fu_2 = interp1d(t_array, theo_stage_res.u[2].y, kind='cubic', fill_value="extrapolate")

        y0 = [prev_real_stage_res.h(step), prev_real_stage_res.lat(step), prev_real_stage_res.long(step), prev_real_stage_res.vn(step),
            prev_real_stage_res.ve(step), prev_real_stage_res.vd(step), prev_real_stage_res.m_fuel(step)] # start value
        w = odeint(real_dyn_func, y0, t_array, args=(fu_0, fu_1, fu_2)) # solve eq.

        # Interpolating states
        real_stage_res.t = interp1d(t_array, t_array, kind='cubic', fill_value="extrapolate")
        real_stage_res.h = interp1d(t_array, w[:,0], kind='cubic', fill_value="extrapolate")
        real_stage_res.lat = interp1d(t_array, w[:,1], kind='cubic', fill_value="extrapolate")
        real_stage_res.long = interp1d(t_array, w[:,2], kind='cubic', fill_value="extrapolate")
        real_stage_res.vn = interp1d(t_array, w[:,3], kind='cubic', fill_value="extrapolate")
        real_stage_res.ve = interp1d(t_array, w[:,4], kind='cubic', fill_value="extrapolate")
        real_stage_res.vd = interp1d(t_array, w[:,5], kind='cubic', fill_value="extrapolate")
        real_stage_res.m_fuel = interp1d(t_array, w[:,6], kind='cubic', fill_value="extrapolate")
        real_stage_res.u = [fu_0, fu_1, fu_2]

        return real_stage_res

    ocp.dynamics[0] = theo_dyn_func 
    
    ocp.terminal_constraints[0] = term_constr
    ocp.terminal_costs[0] = term_cost
    ocp.path_constraints[0] = path_constr

    ocp.validate()

    ###################################

    # Running first iteration
    mpo = mp_l.mpopt_adaptive(ocp, 6, 4) #6.4
    mpo.colloc_scheme = 'LGR'
    sol = mpo.solve()
    post = mpo.process_results(sol,plot=False)

    # Getting optimal results
    stage_res=Stage_result()
    stage_res = get_variables(post, stage_res)

    # Running with real dynamics 
    real_stage_res=Stage_result()
    real_stage_res = get_variables(post, real_stage_res)
    real_stage_res = init_real_flight(stage_res, stage_res, 0)

    # Creating an object for output and saving initial values
    real_stage_res_output = Stage_result()
    real_stage_res_output.add_states(real_stage_res,0)

    # Setting time settings for integration
    time = simulation_step # Starting from t = step
    step = simulation_step # Setting dt

    # Starting simulation
    while step<=stage_res.t.y[-1]: 
        
        # Setting new initial conditions for reinitialized solver
        #            [ x[0],    x[1],   x[2],  x[3],   x[4],   x[5],    x[6],   x[7]  ]
        #            [   h,      lat,   long,   Vn,     Ve,     Vd,    m_dot,         ]
        ocp.terminal_constraints[0] = lambda xf, tf, x0, t0: [x0[0] - real_stage_res.h(step), 
                                                              x0[1] - real_stage_res.lat(step),
                                                              x0[2] - real_stage_res.long(step), 
                                                              x0[3] - real_stage_res.vn(step),
                                                              x0[4] - real_stage_res.ve(step), 
                                                              x0[5] - real_stage_res.vd(step), 
                                                              x0[6] - real_stage_res.m_fuel(step), 
                                                              
                                                              xf[0] - target[0],
                                                              xf[1] - target[1],
                                                              xf[2] - target[2],
                                                              xf[3] - target[3], 
                                                              xf[4] - target[4], 
                                                              xf[5] - target[5],
                                                              xf[6] - target[6]]
        
        ocp.x00[0] = [real_stage_res.h(step), real_stage_res.lat(step), real_stage_res.long(step), real_stage_res.vn(step),
                      real_stage_res.ve(step), real_stage_res.vd(step), real_stage_res.m_fuel(step)] #starting conditions
        ocp.validate()

        # Running reinitialized solver and getting results
        mpo._ocp = ocp
        sol = mpo.solve(sol, reinitialize_nlp=True)
        stage_res = Stage_result()
        post = mpo.process_results(sol, plot=False)
        stage_res = get_variables(post, stage_res)

        # Running with real dynamics
        temp_stage_res = real_stage_res
        real_stage_res = Stage_result()
        real_stage_res = init_real_flight(stage_res, temp_stage_res, step)
        real_stage_res_output.add_states(real_stage_res,time)

    x_output = [real_stage_res_output.h, real_stage_res_output.lat, real_stage_res_output.long, 
                real_stage_res_output.vn, real_stage_res_output.ve, real_stage_res_output.vd, 
                real_stage_res_output.m_fuel]
    u_output = real_stage_res_output.u
    t_output = real_stage_res_output.t
    return (x_output, u_output, t_output)