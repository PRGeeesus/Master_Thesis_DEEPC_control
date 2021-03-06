import time
import numpy as np
import csv
from numpy.lib.histograms import histogram_bin_edges

import math
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import random
import osqp
import scipy as sp
from scipy import sparse

class ISystem:
    def __init__(self,x_0,y_0,gain):
        self.x0 = x_0
        self.y0 = y_0
        self.T = gain
        self.x = self.x0
        self.y = self.y0
        self.u = 0
        self.SystemHistory = np.array([[self.y0,self.x0]])
    
    def resetSystem(self):
        self.x = self.x0
        self.y = self.y0
        self.clearHistory()

    def clearHistory(self):
        self.SystemHistory = np.array([[self.y0,self.x0]])

    def truncateInput(self,inp):
        temp = 0
        if inp < -1:
            temp = -1
        if inp > 1:
            temp = 1
        if inp > -1 and inp < 1:
            temp = inp
        return temp

    def OneTick(self,input):
        #inputt = self.truncateInput(input)
        inputt = input
        self.u = inputt
        self.y = self.x + (self.T * inputt) + (random.random())*0.01
        self.x = self.y
        self.SystemHistory = np.vstack((self.SystemHistory,[inputt,self.y]))
        return self.y

# MPC BY HAND not finished
class CessnaSystem():
    def __init__(self,SAMPLETIME) -> None:
        print("Init CHessna System")
        self.T_SAMPLE = SAMPLETIME
        self.A = np.asarray([[-1.288,0,0.98,0],[0,0,1,0],[-5.4293,0,-1.8366,0],[-128.2,128.2,0,1]])
        self.B = np.asarray([[-0.3],[0],[-17],[0]])
        self.C = np.asarray([[0,1,0,0],[0,0,0,1]])
        self.u_constr = np.asarray([-0.262,0.262]) #-15° and 15°
        self.u_dot_constr = np.asarray([-0.524,0.524]) # -60° and 60°
        self.x_1_constr = np.asarray([-0.349,0.349]) #pitch angle limited to -39° and 39° 
        #x1: angle of attack, x2: pitch angle, x3: pitch rate, x4: altitude
        self.x = np.asarray([[0.0],[0.0],[0.0],[5000.0]])
        self.y = np.asarray([[0.0],[5000.0]])
        #print(np.shape(self.x))
        #print(np.shape(np.matmul(self.A,self.x)),np.shape(np.matmul(self.B,[[0]])))
        self.SystemHistory = np.array([[0,5000,0]])
        print("Init Done")

    def OneTick(self,input):
        u = np.asarray([input])
        #print(np.shape(self.x))
        
        self.x = (np.matmul(self.A,self.x) + np.matmul(self.B,[u]))*self.T_SAMPLE
        self.y = np.matmul(self.C,self.x)*(1/self.T_SAMPLE)
        #print(np.shape(self.C),np.shape(self.x))
        self.SystemHistory = np.vstack((self.SystemHistory,[u[0],self.y[0],self.y[1]]))
        #print(np.shape(u[0].tolist()),np.shape(self.y[0].tolist()),np.shape(self.y[1]),u[0],self.y[0],self.y[1])
        return self.y

class ChessnaMPCController():
    def __init__(self,TIME_HORIZON,SAMPLE_TIME):
        self.System = CessnaSystem()
        self.TIME_HORIZON = TIME_HORIZON
        self.SAMPLE_TIME = SAMPLE_TIME
        
        self.Ad = sparse.csc_matrix(self.System.A)
        self.Bd = sparse.csc_matrix(self.System.B)
        [self.nx, self.nu] = self.Bd.shape
        nx = self.nx
        nu = self.nu
        #print("nx:",nx,"nu:",nu)

        # Constraints
        self.u0 = 0.0
        # self.umin = np.array([-0.262]) - self.u0
        # self.umax = np.array([0.262]) - self.u0
        # self.xmin = np.array([-np.inf,-0.349,-np.inf,-np.inf])
        # self.xmax = np.array([ np.inf,0.349, np.inf, np.inf])
        # self.u_dot_min = np.array([-0.524])
        # self.u_dot_max = np.array([0.524])
        factor = (np.pi/180.0)
        self.umin = np.array([-15*factor]) - self.u0
        self.umax = np.array([15*factor]) - self.u0
        self.xmin = np.array([-np.inf,-39*factor,-np.inf,10.0])
        self.xmax = np.array([ np.inf,39*factor, np.inf, np.inf])
        self.u_dot_min = np.array([-60*factor*(1/SAMPLE_TIME)])
        self.u_dot_max = np.array([60*factor*(1/SAMPLE_TIME)])

        # Objective function
        self.Q = sparse.diags([1.0,1.0,1.0,1.0])
        self.QN = self.Q
        self.R = sparse.diags([10.0])

        # Initial and reference states
        self.x0 = np.array([0.,0.,0.,5000.0])
        self.xr = np.array([0.,0.,0.,0.0])
        self.ur = np.array([0.0])
        self.x = self.x0
        # Prediction horizon
        self.N = self.TIME_HORIZON
        N = self.N
        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        self.P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN,
                            sparse.kron(sparse.eye(self.N),self.R)], format='csc')
        # - linear objective
        self.q = np.zeros((((self.N+1)*self.nx + self.N*self.nu),))

        #self.q = np.hstack([np.kron(np.ones(self.N), -2*self.Q.dot(self.xr)), -2*self.QN.dot(self.xr),
        #            np.zeros(self.N*self.nu)])

        # - linear dynamics
        #matr1 = sparse.eye(N+1).toarray()
        #matr1[0][0] = 0
        #matr1 = sparse.csr_matrix(matr1)
        #self.Ax = sparse.kron(matr1,-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        self.Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        self.Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N*self.nu)), sparse.eye(N)]), self.Bd)
        self.Aeq = sparse.hstack([self.Ax, self.Bu])
        self.leq = np.hstack([[0.0,0.0,0.0,0.0], np.zeros(N*nx)])
        #self.leq = np.hstack([-self.x0, np.zeros(N*nx)])
        self.ueq = self.leq

        # - input and state constraints
        self.Aineq = sparse.eye((N+1)*nx + N*nu)
        #print(np.shape(np.zeros((N*nu,(N+1)*nx)))) #(5,24)
        #print(np.shape((sparse.eye(N*nu)-sparse.eye(N*nu,k=1)).toarray())) #(5,5)

        self.Au_dot_inequality = np.hstack([np.zeros((N*nu,(N+1)*nx)),
                                            (-sparse.eye(N*nu)+sparse.eye(N*nu,k=1)).toarray()])
        #print(np.shape(self.Au_dot_inequality))
        #self.Aineq = np.hstack([sparse.eye((N+1)*nx + N*nu),sparse.eye((N+1)*nx + N*nu)+sparse.eye((N+1)*nx + N*nu,k=1)])
        self.lineq = np.hstack([np.kron(np.ones(N+1), self.xmin), np.kron(np.ones(N), self.umin)])
        self.uineq = np.hstack([np.kron(np.ones(N+1), self.xmax), np.kron(np.ones(N), self.umax)])

        self.l_udot_ineq = np.kron(np.ones(N), self.u_dot_min)
        self.u_udot_ineq = np.kron(np.ones(N), self.u_dot_max)
        # - OSQP constraints
        #print(np.shape(self.Aineq))
        #print(np.shape(self.Au_dot_inequality))
        A = sparse.vstack([self.Aineq,self.Au_dot_inequality])
        self.A = sparse.vstack([self.Aeq, A], format='csc')
        
        self.l = np.hstack([self.lineq,self.l_udot_ineq])
        self.l = np.hstack([self.leq, self.l])
        self.u = np.hstack([self.uineq,self.u_udot_ineq])
        self.u = np.hstack([self.ueq,self.u])

        # Create an OSQP object
        self.prob = osqp.OSQP()
        # Setup workspace
        self.prob.setup(2*self.P, 
                        self.q, 
                        self.A, 
                        self.l, 
                        self.u, 
                        warm_start=False,
                        verbose = False,
                        adaptive_rho = False,
                        max_iter = 100000)
        self.q = self.update_q()
        # Simulate in closed loop

        self.system_outputs = []
        self.system_inputs = []
    
    def update_q(self):
        self.q = np.hstack([np.kron(np.ones(self.N), -2*self.Q.dot(self.xr)), -2*self.QN.dot(self.xr),
                    np.kron(np.ones(self.N), -self.R.dot(self.ur))])
        #self.q = np.zeros((29,))
        #self.q = np.hstack([np.kron(np.ones(self.N), -2*self.Q.dot(self.xr)), -2*self.QN.dot(self.xr),
        #            np.zeros(self.N*self.nu)])
        self.prob.update(q=self.q)
        #print(self.xr)
    def update_SOLL_HIEGHT(self,new_height):
        self.xr = np.array([0.,0.,0.,new_height])
        self.update_q()

    def getPredictionControl(self):
        res = self.prob.solve()
        if res.info.status != 'solved':
                print("status info:",res.info.status)
                raise ValueError('OSQP did not solve the problem!')
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        controls = res.x[(self.N+1)*self.nx:]
        states =   res.x[:(self.N+1)*self.nx]
        controls = np.reshape(controls,(self.N,self.nu))
        states = np.reshape(states,(self.N+1,self.nx))
        
        return controls,states

    def outputFromPredictionState(self,state):
        return np.matmul(self.System.C,state)

    def getSystemResponse(self,input):
         #print(self.x,self.Ad.dot(self.x)," + ",self.Bd.dot(input))
         self.x_next = (self.Ad.dot(self.x) + self.Bd.dot(input))
         y = np.matmul(self.System.C,self.x_next)
         self.x =  self.x_next
         #self.x = (self.Ad.dot(self.x) + self.Bd.dot(input))
         self.system_outputs.append(y)
         self.system_inputs.append(input[0]*(180/np.pi))
         return y
        
    
    def updateSystem(self):
        #self.l[:self.nx] = -self.prev_x
        #self.u[:self.nx] = -self.prev_x
        self.l[:self.nx] = -self.x
        self.u[:self.nx] = -self.x
        self.prob.update(l=self.l, u=self.u)

class InvertedPendulumSS():
    def __init__(self) -> None:
        self.M = .5;
        self.m = 0.2;
        self.b = 0.1;
        self.I = 0.006;
        self.g = 9.8;
        self.l = 0.3;

        self.p = self.I*(self.M+self.m)+self.M*self.m*np.square(self.l) 
        #denominator for the A and B matrices

        self.A = [[0,      1,              0,           0],
            [0, -(self.I+self.m*np.square(self.l))*self.b/self.p,  (np.square(self.m)*self.g*np.square(self.l))/self.p,   0],
            [0,      0,              0,           1],
            [0, -(self.m*self.l*self.b)/self.p,       self.m*self.g*self.l*(self.M+self.m)/self.p,  0]]
        self.B = [     [0],
            [(self.I+self.m*np.square(self.l))/self.p],
                [0],
                [self.m*self.l/self.p]]
        self.C = [[1, 0, 0, 0],
                  [0, 0, 1, 0]]
        self.D = [[0],
                  [0]]
        #print("A:",np.shape(self.A))
        #print("B:",np.shape(self.B))
        #print("C:",np.shape(self.C))
        #print("D:",np.shape(self.D))
        self.x = np.array([[0],[0],[0.0],[0]])
        #print("x:",np.shape(self.x))
        self.u = np.array([0])
        #print("u:",np.shape(self.u))
        self.y = np.array([[0],[0]])
        self.SystemHistory = np.array([[0,0,0]])
    def OneTick(self,input):
        self.u = np.array(input)
        #print("u:",np.shape(self.u))
        #print("Ax:",np.shape(np.matmul(self.A,self.x)))
        #print("Bu:",np.shape(np.matmul(self.B,self.u)))
        self.x = np.matmul(self.A,self.x) + self.B*self.u #np.matmul(self.B,self.u)
        #print("x:",np.shape(self.x))
        self.y = np.matmul(self.C,self.x)
        #print(np.shape(self.y),self.y)
        #print(np.shape(self.y[0]),self.y[0])
        #print(np.shape(self.C),np.shape(self.x))
        self.SystemHistory = np.vstack((self.SystemHistory,[input[0],self.y[0],self.y[1]]))
        #print(np.shape(u[0].tolist()),np.shape(self.y[0].tolist()),np.shape(self.y[1]),u[0],self.y[0],self.y[1])
        return self.y

class InvertedPendulum_MPC():
    def __init__(self,timestep,horizon,SOLLWERT) -> None:
        self.TIMESTEP = timestep
        self.HORIZON_N_SEC = horizon
        self.SOLLWERT = SOLLWERT
        # System matrices
        
        #self.M = .5;
        self.M = .5;
        self.m = 0.2;
        self.b = 0.1;
        self.I = 0.006;
        self.g = 9.8;
        #self.l = 0.3;
        self.l = 1;


        self.p = self.I*(self.M+self.m)+self.M*self.m*np.square(self.l)
        self.A_raw = np.array([[0,      1,              0,           0],
            [0, -(self.I+self.m*np.square(self.l))*self.b/self.p,  (np.square(self.m)*self.g*np.square(self.l))/self.p,   0],
            [0,      0,              0,           1],
            [0, -(self.m*self.l*self.b)/self.p,       self.m*self.g*self.l*(self.M+self.m)/self.p,  0]])

        self.B_raw = np.array([[0],
            [(self.I+self.m*np.square(self.l))/self.p],
                [0],
                [self.m*self.l/self.p]])
        self.C_raw = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
        self.D_raw = [[0],
                  [0]]
        
        
        self.Bd = sparse.csc_matrix(self.B_raw)*self.TIMESTEP

        self.factor_deg_to_rad = (np.pi/180)
        self.factor_rad_to_deg = (180/np.pi)
        self.Cd = sparse.csc_matrix(self.C_raw)
        self.Dd = sparse.csc_matrix(self.D_raw)
        [nx, nu] = self.Bd.shape
        self.nx = nx
        self.nu = nu

        self.Ad = sparse.csc_matrix(self.A_raw)*self.TIMESTEP + sparse.eye(nx)
        # Prediction horizon
        N = self.HORIZON_N_SEC
        self.N = N

        # Constraints

        u0 = 0
        umin = np.array([-10])
        umax = np.array([10])
        xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf])
        #xmax = np.array([np.inf, 0.349,  np.inf, np.inf])
        xmax = xmin*-1
        # Objective function
        Q = sparse.diags([1., 1., 1., 1.])
        QN = Q*10
        R = sparse.diags([10.])

        # Initial and reference states
        self.x0 = np.array([0.2,0,0.1,0])
        self.xr = np.array([self.SOLLWERT,0,0,0])

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(self.xr)), -QN.dot(self.xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)

        # add the input constrain for deltau
        # delta_x = sparse.kron(sparse.eye(N+1),np.zeros((nu,nx)))
        # delta_u = -sparse.eye(N)+sparse.eye(N, k=1)
        # delta_u = sparse.vstack([sparse.csc_matrix((1, N)), delta_u])
        # A_delta = sparse.hstack([delta_x, delta_u])
        

        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

        # delta u constraints
        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        self.prob.setup(P, q, A, self.l, self.u,adaptive_rho = True, warm_start=False,verbose = False)

        # RECORD DATA
        self.out_x = []
        self.inputs = []
        self.prediction = [0]
        self.first_prediction = []
        self.SnapShotFirstPrediction_Flag = 0


    def resetSystem(self):
        self.out_x = []
        self.inputs = []
        self.x0 = np.zeros(self.nx)
    
    def setReference(self,xr):
        self.xr = xr

    def getControl(self):
        res = self.prob.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        #print(res.x)
        #print(self.N,self.nx)
        temp = res.x[:((self.N + 1)*self.nx)]
        temp = np.reshape(temp,(self.N + 1, self.nx))
        predictions = temp[:,0]
        #print(predictions,np.shape(predictions))
        if self.SnapShotFirstPrediction_Flag == 0:
            self.first_prediction.append(predictions)
            self.SnapShotFirstPrediction_Flag = 1
            
        #self.prediction.append(predictions[-1])
        self.prediction.append(predictions[0])
        return ctrl
    def getSystemresponse(self,ctrl):
        #ctrl = np.reshape(ctrl,(len(ctrl),1))
        self.x0 =  self.Ad.dot(self.x0) + self.Bd*ctrl
        

        y = self.Cd.dot(self.x0)

        #self.inputs.append(ctrl[0]*self.factor_rad_to_deg)
        self.inputs.append(ctrl[0])
        self.out_x.append(y[0])
        return y
    def updateSystem(self):
        self.l[:self.nx] = -self.x0
        self.u[:self.nx] = -self.x0
        self.prob.update(l=self.l, u=self.u)

class FederMasseSystem:
    def __init__(self,timestep) -> None:
        # Parameters defining the system
        self.timestep = timestep
        self.c = 4 # Damping constant
        self.k = 2 # Stiffness of the spring
        self.m = 20 # Mass
        self.F = 5 # Force
        # Simulation Parameters
        # System matrices
        self.A = np.array([[0, 1], [-self.k/self.m, -self.c/self.m]])
        self.B = np.array([[0], [1/self.m]])
        self.C = np.array([[1, 0]])
        self.A = np.eye(2) + self.A*self.timestep
        self.B = self.B*self.timestep
        self.C = self.C
        self.x0 = np.array([0.0,0.0])

        self.out_pos = []
        self.out_vel = []
        self.in_force = []

    def changeSystem(self):
        self.A = np.array([[0, 1], [-self.k/self.m, -self.c/self.m]])
        self.B = np.array([[0], [1/self.m]])
        self.C = np.array([[1, 0]])
        self.A = np.eye(2) + self.A*self.timestep
        self.B = self.B*self.timestep
        self.C = self.C

    def resetSystem(self):
        self.x0 = np.array([0.,0.])
        self.out_pos = [self.x0[0]]
        self.out_vel = [self.x0[1]]
        self.in_force = [0]

    def OneTick(self,input_force):
        u = np.array([input_force+(random.random())*0.01])
        self.x0 = np.matmul(self.A,self.x0) + np.matmul(self.B,u)
        self.out_pos.append(self.x0[0])
        self.out_vel.append(self.x0[1])
        self.in_force.append(input_force)
        y = np.matmul(self.C,self.x0)
        return y

#using OSQP
class FederMasseSystem_MPC():
    def __init__(self,timestep,horizon,SOLLWERT) -> None:
        self.TIMESTEP = timestep
        self.HORIZON_N_SEC = horizon
        self.SOLLWERT = SOLLWERT
        # System matrices
        # https://lost-contact.mit.edu/afs/umich.edu/class/ctms/Public/html/examples/pend/invpen.htm
        # https://lost-contact.mit.edu/afs/umich.edu/class/ctms/Public/html/examples/pend/invss.htm
        c = 4 # Damping constant
        k = 2 # Stiffness of the spring
        m = 20 # Mass
        F = 5 # Force
        self.A_raw = np.array([[0, 1], [-k/m, -c/m]])

        self.B_raw = np.array([[0], [1/m]])
        self.C_raw = np.array([[1, 0]])
        
        self.Ad = sparse.csc_matrix(self.A_raw)
        self.Bd = sparse.csc_matrix(self.B_raw)*self.TIMESTEP

        self.factor_deg_to_rad = (np.pi/180)
        self.factor_rad_to_deg = (180/np.pi)
        self.Cd = sparse.csc_matrix(self.C_raw)
        [nx, nu] = self.Bd.shape
        self.nx = nx
        self.nu = nu
        self.Ad =  self.Ad*self.TIMESTEP + sparse.eye(nx)
        # Prediction horizon
        N = self.HORIZON_N_SEC
        self.N = N

        # Constraints

        u0 = 0
        umin = np.array([-20])
        umax = np.array([20])
        xmin = np.array([-np.inf,-np.inf])
        #xmax = np.array([np.inf, 0.349,  np.inf, np.inf])
        xmax = xmin*-1
        # Objective function
        Q = sparse.diags([100., 100.])
        QN = Q*100
        R = sparse.diags([1.])

        # Initial and reference states
        self.x0 = np.zeros(nx)
        self.xr = np.array([self.SOLLWERT,0])

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(self.xr)), -QN.dot(self.xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)

        # add the input constrain for deltau
        # delta_x = sparse.kron(sparse.eye(N+1),np.zeros((nu,nx)))
        # delta_u = -sparse.eye(N)+sparse.eye(N, k=1)
        # delta_u = sparse.vstack([sparse.csc_matrix((1, N)), delta_u])
        # A_delta = sparse.hstack([delta_x, delta_u])
        

        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

        # delta u constraints
        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        self.prob.setup(P, q, A, self.l, self.u,adaptive_rho = True, warm_start=True,verbose = False)

        # RECORD DATA
        self.out_x = []
        self.inputs = []
        self.prediction = [0]
        self.first_prediction = []
        self.SnapShotFirstPrediction_Flag = 0


    def resetSystem(self):
        self.out_x = []
        self.inputs = []
        self.x0 = np.zeros(self.nx)
    
    def setReference(self,xr):
        self.xr = xr

    def getControl(self):
        res = self.prob.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        #print(res.x)
        #print(self.N,self.nx)
        temp = res.x[:((self.N + 1)*self.nx)]
        temp = np.reshape(temp,(self.N + 1, self.nx))
        predictions = temp[:,0]
        #print(predictions,np.shape(predictions))
        if self.SnapShotFirstPrediction_Flag == 0:
            self.first_prediction.append(predictions)
            self.SnapShotFirstPrediction_Flag = 1
            
        #self.prediction.append(predictions[-1])
        self.prediction.append(predictions[0])

        return ctrl
    def getSystemresponse(self,ctrl):
        #ctrl = np.reshape(ctrl,(len(ctrl),1))
        noise = np.asarray(np.random.normal(self.x0,0.01,2))
        noise[1] = 0
        self.x0 =  self.Ad.dot(self.x0) + self.Bd*ctrl #+noise
        

        y = self.Cd.dot(self.x0)

        #self.inputs.append(ctrl[0]*self.factor_rad_to_deg)
        self.inputs.append(ctrl[0])
        self.out_x.append(y[0])
        return y
    def updateSystem(self):
        self.l[:self.nx] = -self.x0
        self.u[:self.nx] = -self.x0
        self.prob.update(l=self.l, u=self.u)


class QuadCopter():
    def __init__(self,t_s):
        self.TIMESTEP = t_s
        Ad = sparse.csc_matrix([
            [1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ],
            [0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ],
            [0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ],
            [0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ],
            [0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ],
            [0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992],
            [0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ],
            [0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ],
            [0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ],
            [0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ],
            [0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ],
            [0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846]
            ])
        Bd = sparse.csc_matrix([
            [0.,      -0.0726,  0.,     0.0726],
            [-0.0726,  0.,      0.0726, 0.    ],
            [-0.0152,  0.0152, -0.0152, 0.0152],
            [-0.,     -0.0006, -0.,     0.0006],
            [0.0006,   0.,     -0.0006, 0.0000],
            [0.0106,   0.0106,  0.0106, 0.0106],
            [0,       -1.4512,  0.,     1.4512],
            [-1.4512,  0.,      1.4512, 0.    ],
            [-0.3049,  0.3049, -0.3049, 0.3049],
            [-0.,     -0.0236,  0.,     0.0236],
            [0.0236,   0.,     -0.0236, 0.    ],
            [0.2107,   0.2107,  0.2107, 0.2107]])
        [nx, nu] = Bd.shape

        # Constraints
        u0 = 10.5916
        umin = np.array([9.6, 9.6, 9.6, 9.6]) - u0
        umax = np.array([13., 13., 13., 13.]) - u0
        xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
                        -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
        xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
                        np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

        # Objective function
        Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
        QN = Q
        R = 0.1*sparse.eye(4)

        # Initial and reference states
        x0 = np.zeros(12)
        xr = np.array([0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

        # Prediction horizon
        N = 10

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)
        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace
        prob.setup(P, q, A, l, u, warm_start=True,verbose = False)

        # Simulate in closed loop
        nsim = 15
        self.system_behaviour = []
        self.inputs = []
        self.time = []
        for i in range(nsim):
            # Solve
            self.time.append(i)
            res = prob.solve()

            # Check solver status
            if res.info.status != 'solved':
                raise ValueError('OSQP did not solve the problem!')

            # Apply first control input to the plant
            ctrl = res.x[-N*nu:-(N-1)*nu]
            x0 = Ad.dot(x0) + Bd.dot(ctrl)
            print(x0)
            self.inputs.append(ctrl)
            self.system_behaviour.append(x0[2])

            # Update initial state
            l[:nx] = -x0
            u[:nx] = -x0
            prob.update(l=l, u=u)

# MPC with OSQP
class Chessna2():
    def __init__(self,TIMESTEP,time_horizon_s):
    
        self.TIMESTEP = TIMESTEP
        self.HORIZON_N_SEC = time_horizon_s

        self.Ad = sparse.csc_matrix([
            [-1.2822, 0 ,0.98, 0 ],
            [0, 0, 1, 0],
            [-5.4293, 0, -1.8366, 0 ],
            [128.2, 128.2, 0, 0]])


        
        self.Bd = sparse.csc_matrix([
            [-0.3],
            [0],
            [   -17],
            [0,]])*self.TIMESTEP
        
        self.factor_deg_to_rad = (np.pi/180)
        self.factor_rad_to_deg = (180/np.pi)
        self.Cd = sparse.csc_matrix([[0, 1*self.factor_rad_to_deg ,0, 0 ],
                                    [0, 0, 0, 1],])
        [nx, nu] = self.Bd.shape
        self.nx = nx
        self.nu = nu
        self.Ad =  self.Ad*self.TIMESTEP + sparse.eye(nx)
        # Prediction horizon
        N = self.HORIZON_N_SEC
        self.N = N

        # Constraints

        u0 = 0
        umin = np.array([-0.262])
        umax = np.array([0.262])
        xmin = np.array([-np.inf,-0.349,-np.inf, 100.0])
        xmax = np.array([np.inf, 0.349,  np.inf, np.inf])

        deltau_min = np.array([-0.524])*self.TIMESTEP
        deltau_max = np.array([0.524])*self.TIMESTEP

        # Objective function
        Q = sparse.diags([1., 1., 1., 1.])
        QN = Q*10
        R = sparse.diags([1.])

        # Initial and reference states
        #self.x0 = np.zeros(nx)
        self.x0 = np.array([0.,0.,0.,5000.,])
        self.xr = np.array([0.,0.,0.,5020.,])

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(self.xr)), -QN.dot(self.xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)

        # add the input constrain for deltau
        delta_x = sparse.kron(sparse.eye(N+1),np.zeros((nu,nx)))
        delta_u = -sparse.eye(N)+sparse.eye(N, k=1)
        delta_u = sparse.vstack([sparse.csc_matrix((1, N)), delta_u])
        A_delta = sparse.hstack([delta_x, delta_u])
        

        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

        # delta u constraints
        l_deltau = np.kron(np.ones(N), deltau_min)      
        u_deltau = np.kron(np.ones(N), deltau_max)
        l_deltau = np.hstack([0.0,l_deltau])
        u_deltau = np.hstack([0.0,u_deltau])
        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq,A_delta], format='csc')
        self.l = np.hstack([leq, lineq,l_deltau])
        self.u = np.hstack([ueq, uineq,u_deltau])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        self.prob.setup(P, q, A, self.l, self.u,adaptive_rho = False, warm_start=False,verbose = False)

        # RECORD DATA
        self.out_alt = []
        self.out_angle = []
        self.inputs = []
        self.time = []

    def resetSystem(self):
        self.out_alt = []
        self.out_angle = []
        self.inputs = []
        self.time = []
        self.x0 = np.array([0.,0.,0.,5000.,])
        self.xr = np.array([0.,0.,0.,5020.,])


    def getControl(self):
        res = self.prob.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        #print(res.x,ctrl)
        return ctrl

    def getSystemresponse(self,ctrl):
        #ctrl = np.reshape(ctrl,(len(ctrl),1))
        self.x0 =  (self.Ad.dot(self.x0) + self.Bd*ctrl)

        y = self.Cd.dot(self.x0)

        #self.inputs.append(ctrl[0]*self.factor_rad_to_deg)
        self.inputs.append(ctrl[0])
        self.out_alt.append(y[1])
        self.out_angle.append(y[0])
        return y
        
    def updateSystem(self):
        self.l[:self.nx] = -self.x0
        self.u[:self.nx] = -self.x0
        self.prob.update(l=self.l, u=self.u)

class InvertedPendulum_WithCart():
    def __init__(self,timestep,horizon) -> None:
        self.TIMESTEP = timestep
        self.HORIZON_N_SEC = horizon
        # System matrices
        # https://lost-contact.mit.edu/afs/umich.edu/class/ctms/Public/html/examples/pend/invpen.htm
        # https://lost-contact.mit.edu/afs/umich.edu/class/ctms/Public/html/examples/pend/invss.htm
        self.A_raw = np.array([[0, 1, 0, 0],
                      [0, -0.1818, 2.6727, 0],
                      [0, 0, 0, 1],
                      [0, -0.4545, 31.1818, 0]])

        self.B_raw = np.array([[0], [1.8182],[0],[4.5455]])
        self.C_raw = np.array([[1, 0,0,0],[0, 0,1,0]])
        
        self.Ad = sparse.csc_matrix(self.A_raw)
        self.Bd = sparse.csc_matrix(self.B_raw)*self.TIMESTEP

        self.factor_deg_to_rad = (np.pi/180)
        self.factor_rad_to_deg = (180/np.pi)
        self.Cd = sparse.csc_matrix(self.C_raw)
        [nx, nu] = self.Bd.shape
        self.nx = nx
        self.nu = nu
        self.Ad =  self.Ad*self.TIMESTEP + sparse.eye(nx)
        # Prediction horizon
        N = self.HORIZON_N_SEC
        self.N = N

        # Constraints

        u0 = 0
        umin = np.array([-10])
        umax = np.array([10])
        xmin = np.array([-90*self.factor_deg_to_rad,-90*self.factor_deg_to_rad,-90*self.factor_deg_to_rad, -90*self.factor_deg_to_rad])
        #xmax = np.array([np.inf, 0.349,  np.inf, np.inf])
        xmax = xmin*-1
        # Objective function
        Q = sparse.diags([1., 1., 1., 1.])
        QN = Q*10
        R = sparse.diags([1.])

        # Initial and reference states
        self.x0 = np.zeros(nx)
        self.xr = np.array([2,0,0,0])

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(self.xr)), -QN.dot(self.xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)

        # add the input constrain for deltau
        # delta_x = sparse.kron(sparse.eye(N+1),np.zeros((nu,nx)))
        # delta_u = -sparse.eye(N)+sparse.eye(N, k=1)
        # delta_u = sparse.vstack([sparse.csc_matrix((1, N)), delta_u])
        # A_delta = sparse.hstack([delta_x, delta_u])
        

        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

        # delta u constraints
        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        self.prob.setup(P, q, A, self.l, self.u,adaptive_rho = False, warm_start=False,verbose = False)

        # RECORD DATA
        self.out_x = []
        self.out_angle = []
        self.inputs = []


    def resetSystem(self):
        self.out_x = []
        self.out_angle = []
        self.inputs = []
        self.x0 = np.zeros(self.nx)
    
    def setReference(self,xr):
        self.xr = xr

    def getControl(self):
        res = self.prob.solve()
        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        return ctrl
    def getSystemresponse(self,ctrl):
        #ctrl = np.reshape(ctrl,(len(ctrl),1))
        self.x0 =  (self.Ad.dot(self.x0) + self.Bd*ctrl)

        y = self.Cd.dot(self.x0)

        #self.inputs.append(ctrl[0]*self.factor_rad_to_deg)
        self.inputs.append(ctrl[0])
        self.out_x.append(y[0])
        self.out_angle.append(y[1])
        return y
    def updateSystem(self):
        self.l[:self.nx] = -self.x0
        self.u[:self.nx] = -self.x0
        self.prob.update(l=self.l, u=self.u)

def saveAsCSV(filename,data):
    print(len(data)," datapoints collected")
    print("Saving recorded data in csv file: " + filename + ".csv")
    with open(filename+'.csv', 'w') as f:
              write = csv.writer(f)
              write.writerows(data)
    np.savetxt(filename + "_numpy.csv", 
           data,
           delimiter =", ", 
           fmt ='% s')

def readFromCSV(filename):
    return np.genfromtxt(filename+".csv", delimiter=",")

# mean between prediction and actual system output
def Evaluate_Tracking_Accuarcy(system_output,controller_prediction):
    if len(system_output) != len(controller_prediction):
        print("Can not evaluate Tracking AAAccuary. Unequal array sizes",np.shape(system_output),np.shape(controller_prediction))
        return 0,0
    else:
        difference = []
        for i in range(len(system_output)):
            difference.append(np.abs(system_output[i] - controller_prediction[i]))
        mean = np.mean(difference)
        stddev = np.std(difference)
    #print("Prediction to output mean:",'%.3f'%mean,"std",'%.3f'%stddev,"End difference:",'%.3f'%(system_output[-1]-controller_prediction[-1]))
    return mean, stddev

# mean between soll and system state
def Evaluate_Control_Accuarcy(sollwert,system_output):
    if len(sollwert) != len(system_output):
        print("Can not evaluate Tracking Accuary. Unequal array sizes",np.shape(sollwert),np.shape(system_output))
    else:
        difference = []
        for i in range(len(system_output)):
            difference.append(np.abs(system_output[i] - sollwert[i]))
        mean = np.mean(difference)
        stddev = np.std(difference)
    print("Soll zu output mean:",'%.3f'%mean,"std",'%.3f'%stddev,"End difference:",'%.3f'%(sollwert[-1]-system_output[-1]))
    return mean, stddev

def SaveSystemSettings(filename,system,controller):
    print("Saving recorded data in csv file: " + filename + ".txt")
    with open(filename+'.txt', 'w') as f:
        f.write("Attribute : value" +'\n')
        f.write('\n')
        f.write("Controller Settings:" +'\n')
        f.write('\n')
        f.write("Controller init data length:" + str(controller.data_length)+'\n')
        f.write("Controller input size:      " + str(controller.input_size)+'\n')
        f.write("Controller output size:     " + str(controller.output_size)+'\n')
        f.write("Controller lambda_s:        " + str(controller.lambda_s)+'\n')
        f.write("Controller lambda_g:        " + str(controller.lambda_g)+'\n')
        f.write("Controller T_ini:           " + str(controller.T_ini)+'\n')
        f.write("Controller T_f:             " + str(controller.T_f)+'\n')
        f.write("Controller Q:               " + str(controller.Q)+'\n')
        f.write("Controller R:               " + str(controller.R)+'\n')
        f.write("lower output constrainat:   " + str(controller.out_constr_lb)+'\n')
        f.write("upper output constrainat:   " + str(controller.out_constr_ub)+'\n')
        f.write("lower input constrainat:    " + str(controller.in_constr_lb)+'\n')
        f.write("upper input constrainat:    " + str(controller.in_constr_ub)+'\n')
        f.write("Redularized:                " + str(controller.regularize)+'\n')
        f.write('\n')
        f.write("System Settings:" +'\n')
        f.write('\n')
        f.write("System type:               " + str(system.__class__)+'\n')
        f.write("System class name:         " + str(system.__class__.__name__)+'\n')

        f.write('\n\n\n' + "Data :          "+str(controller.data)+'\n')

        #write = csv.writer(f)
        #write.write("Attribute 1:" + str(99))
    #np.savetxt(filename + ".txt", 
    #       data,
    #       delimiter =", ", 
    #       fmt ='% s')


