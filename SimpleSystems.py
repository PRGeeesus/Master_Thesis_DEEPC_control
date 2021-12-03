import numpy as np

class SimpleSystem1:
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
        self.y = self.x + self.T * inputt
        self.x = self.y
        self.SystemHistory = np.vstack((self.SystemHistory,[inputt,self.y]))
        return self.y


class CessnaSystem():
    def __init__(self) -> None:
        print("Init CHessna System")
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
        
        self.x = np.matmul(self.A,self.x) + np.matmul(self.B,[u])
        self.y = np.matmul(self.C,self.x)
        #print(np.shape(self.C),np.shape(self.x))
        self.SystemHistory = np.vstack((self.SystemHistory,[u[0],self.y[0],self.y[1]]))
        #print(np.shape(u[0].tolist()),np.shape(self.y[0].tolist()),np.shape(self.y[1]),u[0],self.y[0],self.y[1])
        return self.y
    

import osqp
import scipy as sp
from scipy import sparse

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

class InvertedPendulimSS():
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

import math

# Tiefpass 2. Ordnung
class SecondOrderSystem: 
    def __init__(self, d1, d2):
        if d1 > 0:
            e1 = -1.0 / d1
            x1 = math.exp(e1)
        else: x1 = 0
        if d2 > 0:
            e2 = -1.0 / d2
            x2 = math.exp(e2)
        else: x2 = 0
        a = 1.0 - x1    # b = x1
        c = 1.0 - x2    # d = x2
        self.ac = a * c
        self.bpd = x1 + x2
        self.bd = x1 * x2
        self.Yppr = 0
        self.Ypr = 0
 
    def __call__(self, X): 
        Y = self.ac * X + self.bpd * self.Ypr - self.bd * self.Yppr
        self.Yppr = self.Ypr
        self.Ypr = Y
        return Y

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import control

class FederMasseSystem:
    def __init__(self) -> None:
        # Parameters defining the system
        c = 4 # Damping constant
        k = 2 # Stiffness of the spring
        m = 20 # Mass
        F = 5 # Force
        # Simulation Parameters
        tstart = 0
        tstop = 60
        increment = 0.1
        t = np.arange(tstart,tstop+1,increment)
        # System matrices
        A = [[0, 1], [-k/m, -c/m]]
        B = [[0], [1/m]]
        C = [[1, 0]]
        sys = control.ss(A, B, C, 0)
        t, y, x = control.forced_response(sys, t, F,return_x=True)
        # Step response for the system
        y_list = np.array([])
        x_list =[]
        for i in range(1):
            t, y, x = control.forced_response(sys, t, F,[x[0][-1],x[1][-1]],return_x=True)
            #print("y",np.shape(y),y,type(y))
            y_list = np.hstack((y,y_list))
            #y_list.append(np.array(y).flatten())
            #x_list.append(x,y)
        np.flipud(y_list)
        plt.plot( y_list)
        plt.title('Simulation of Mass-Spring-Damper System')
        plt.xlabel('t')
        plt.ylabel('x(t)')
        plt.grid()
        plt.show()

