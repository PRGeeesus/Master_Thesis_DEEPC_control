import osqp
import sys
import numpy as np
import scipy.optimize as opt
from scipy import sparse

def p_name_shape(func):                                                                                     
    def inner(objs):                                                                                         
        print(str(objs)," shape: ",np.shape(objs),objs)                                                     
        return func(objs)                                                                                                                                                                                 
    return inner            

class Controller:
    # controller has to be initalized with data:
    def __init__(self,data,T_ini,T_f,input_size,output_size):
        self.data_length = len(data)
        self.data = np.asarray(data)
        
        if len(self.data[0]) != (input_size + output_size):
            print("initalization data dimenstion does not match iniatlized in/out sizes: ",len(self.data[0])," != ",input_size," + ",output_size)

        self.T_ini = T_ini
        self.T_f = T_f
        self.L = self.T_ini + self.T_f

        self.input_size = input_size
        self.output_size = output_size

        # defines:
        self.u_r = np.array([0.1,0.0,0.0]) # steady state reference inout is no throttle, steer or brake
        self.y_r = np.array([1.0,2.0,3.0]) # steady state reference output should probably be the reference waypoint I want to reach ?

        #self.Q = np.matrix([[40,0,0],[0,40,0],[0,0,40]]) #tracking error cost Martix taken from paper
        self.Q = np.asarray([[1,0,0],[0,1,0],[0,0,1]])
        #self.R = np.matrix([[160,0,0],[0,4,0],[0,0,4]])  # quardatic control effort cost
        self.R = np.asarray([[1,0,0],[0,1,0],[0,0,1]])
        
        #to be updated as the controller runs:

        self.y_ini = np.zeros((self.output_size)) # T_ini most recent input/output measures
        self.u_ini = np.empty((self.input_size))

        # data is in the form:
        #[throttle,steer,brake,car-x,car-y,car-yaw]
        #[u1,u2,u3, y1,y2,y3]
        self.output_sequence = self.data[:,self.input_size:self.output_size + self.input_size]# y
        self.input_sequence =  self.data[:,:self.input_size] # u


        self.U_p_f = self.generateHankel_Collums(self.L,self.input_sequence)
        self.U_p = self.U_p_f[:self.T_ini*self.input_size,:] 
        self.U_f = self.U_p_f[self.L*self.input_size-self.T_f*self.input_size:,:]

        self.Y_p_f = self.generateHankel_Collums(self.L,self.output_sequence)
        self.Y_p = self.Y_p_f[:self.T_ini*self.output_size,:] 
        self.Y_f = self.Y_p_f[self.L*self.output_size-self.T_f*self.output_size:,:]

        # these are only needed for Regularization:
        self.g_r = []
        self.recalculate_g_r()


        self.lambda_s = 0.1
        self.lambda_g = 10

        # all the initalization for the SOlver:
        init_value = 0.5   
        self.u_len = self.T_f*self.input_size
        self.y_len =  self.T_f*self.output_size
        self.g_len = len(self.input_sequence) - self.L

        self.x0 = np.full(((self.u_len + self.y_len + self.g_len)),init_value)

        # for initalization make u and y ini the Tini last measurements
        # this will get updated once the Vehicle starts to produce its own data
        for i in range(self.T_ini):
            #self.updateIn_Out_Measures(self.input_sequence[self.data_length-1 - self.T_ini + i], self.output_sequence[self.data_length-1 - self.T_ini + i])
            self.updateIn_Out_Measures(self.input_sequence[i], self.output_sequence[i])
        
        # these nees y_ini and u_ini to be filled corretly by the above code
        self.P = self.calculate_P()
        self.q = self.calculate_q()
        self.A = self.calculate_A()

        self.output_constrains_lb = [0,0,-180.0]
        self.output_constrains_ub = [0,0,180.0]
        self.input_constrains_lb  = [0,-1.0,0.0]
        self.input_constrains_ub = [1.0,1.0,1.0]
        
        self.calculate_bounds_u_l()
        
    def update_P(self):
        self.P = self.calculate_P()
    
    def update_q(self):
        self.q = self.calculate_q()

    # update only if Y_p, lambda_g, lambda_s, Q, R changes
    def calculate_P(self):
        """
        Calculates the matrix P for the Optimization problem

        Returns
        -------
        out : matrix q
            Matrix of given shape.
        Notes
        -------
        [Q] [0] [0] \n
        [0] [R] [0] \n
        [0] [0] [l_s * Y_p*Y_p + l_g * I]\n
        """
        P_u = np.kron(np.eye(self.T_f), self.Q) # (15, 15)
        P_y = np.kron(np.eye(self.T_f), self.R) # (15, 15)
        P_g = self.lambda_s * np.matmul(self.Y_p.T,self.Y_p) + np.eye(self.g_len)*self.lambda_g # (43, 43)
        #print("P_u shape :",np.shape(P_u))
        #print("P_y shape :",np.shape(P_y))
        #print("P_g shape :",np.shape(P_g),P_g)
        #print("x0 shape :",np.shape(x0))
        # TODO: THese shapes are STATIC!!!
        zeros_15_15 = np.zeros((self.T_f*self.output_size, self.T_f*self.output_size))
        zeros_15_43 = np.zeros((self.T_f*self.output_size,self.data_length-self.L))
        zeros_43_15 = np.zeros((self.data_length-self.L, self.T_f*self.output_size))
        P = np.block([[P_u,zeros_15_15,zeros_15_43],
                 [zeros_15_15,P_y,zeros_15_43],
                 [zeros_43_15,zeros_43_15,P_g]])
        P = sparse.csc_matrix(P)
        return P

    # update when y_ini, y_r, u_r, Q, R, lambda_g, lambda_s changes
    # in update_y_r
    # in update_in_out_measures
    def calculate_q(self):
        """
        Calculates the matrix q for the Optimization problem

        Returns
        -------
        out : matrix q
            Matrix of given shape.

        """
        temp1 = np.matmul(2.*np.array(self.y_r),self.Q)# TODO: maybe -
        q_u = temp1
        for i in range(self.T_f-1):
            q_u = np.hstack((temp1,q_u))
        temp2 = np.matmul(2.*np.array(self.u_r),self.R) # TODO: maybe -
        q_y = temp2
        for i in range(self.T_f-1):
            q_y = np.hstack((temp2,q_y))
        #print(q_y)
        #print("self.y_ini",np.shape(self.y_ini)," self.Y_p",np.shape(self.Y_p))
        q_g_first_term = np.matmul(self.y_ini.flatten().T,self.Y_p)*2.0*self.lambda_s # TODO: maybe -
        q_g_second_term = self.lambda_g*2.0*self.g_r.T # TODO: maybe -
        #print("q_g_first_term",np.shape(q_g_first_term))
        #print("q_g_second_term",np.shape(q_g_second_term))
        q_g = q_g_first_term + q_g_second_term
        
        q = np.hstack((q_y,q_g))
        q = np.hstack((q_u,q))
        #print("q shape: ",np.shape(q))
        return q

    def calculate_A(self):
        u_ZEROS_1_1 = np.zeros((self.input_size*self.T_ini,self.input_size*self.T_f))
        y_ZEROS_1_2 = np.zeros((self.output_size*self.T_ini,self.output_size*self.T_f))

        u_factor_2_1 = np.eye(self.input_size*self.T_f)*-1.0
        u_ZEROS_2_2 = np.zeros((self.input_size*self.T_f,self.output_size*self.T_f))

        y_ZEROS_3_1 = np.zeros((self.output_size*self.T_f,self.input_size*self.T_f))
        y_factor_3_2 = np.eye(self.output_size*self.T_f)*-1.0
        #print(np.shape(u_ZEROS_1_1),np.shape(y_ZEROS_1_2),np.shape(self.U_p))
        #print(np.shape(u_factor_2_1),np.shape(u_ZEROS_2_2),np.shape(self.U_f))
        #print(np.shape(y_ZEROS_3_1),np.shape(y_factor_3_2),np.shape(self.Y_f))

        A = np.block([[u_ZEROS_1_1,y_ZEROS_1_2,self.U_p],
                     [u_factor_2_1,u_ZEROS_2_2,self.U_f],
                     [y_ZEROS_3_1,y_factor_3_2,self.Y_f]])
        A = sparse.csc_matrix(A)
        return A

    def calculate_bounds_u_l(self):

        # bounds to make it for the description
        upper_bound = 0.0
        zeros_u = [[upper_bound] for i in range(self.input_size*self.T_f)]
        zeros_y = [[upper_bound] for i in range(self.output_size *self.T_f)]
        u_ini_flat = np.reshape(self.u_ini,(self.input_size*self.T_ini,1))
        l = np.vstack((zeros_u,zeros_y))
        l = np.vstack((u_ini_flat,l))
        print("l: ",np.shape(l),l)
        
        self.u = l
        self.l = l

        l = np.array([upper_bound])
        u = np.array([upper_bound])

        # bounds to constrain u and i
        for i in range(1,self.data_length-self.L):
            l = np.vstack((l, [upper_bound]))
            u = np.vstack((u,  [upper_bound]))

        # y
        for i in range(self.T_f):
            l = np.vstack((l,  np.reshape(self.output_constrains_lb, (self.output_size,1))))
            u = np.vstack((u,  np.reshape(self.output_constrains_ub, (self.output_size,1))))
        # u
        for i in range(self.T_f):
            l = np.vstack((l,  np.reshape(self.input_constrains_lb, (self.input_size,1))))
            u = np.vstack((u,  np.reshape(self.input_constrains_ub, (self.input_size,1))))
        
        self.u = self.u + u
        self.l = self.l + l

    # TODO: adapt
    def getInputOutputPrediction(self,verbose = False):
        x0_result = self.solve_for_x_regularized()

        g = x0_result[self.T_f*self.input_size + self.T_f*self.output_size : ]
        u = x0_result[:self.T_f*self.input_size]
        u_star = np.reshape(np.matmul(self.U_f,g),(self.T_f,self.input_size))
        
        y =  x0_result[self.T_f*self.input_size : self.T_f*self.input_size + self.T_f*self.output_size]
        y_star = np.reshape(np.matmul(self.Y_f,g),(self.T_f,self.output_size))

        u = np.reshape(u, (self.T_f,self.input_size))
        u_star = np.reshape(u_star, (self.T_f,self.input_size))
        y = np.reshape(y, (self.T_f,self.output_size))
        y_star = np.reshape(y_star, (self.T_f,self.output_size))

        if verbose:
            self.test_erg(x0_result)
            print("u_star: ",u_star[0],"y_star: ",y_star[0])

        return u,y,u_star,y_star,g
    
    def test_erg(self,x_erg):
        x0_result = x_erg
        x0_t = x0_result
        x0 = x0_result
        x0_t = np.reshape(x0_t, (1,len(x_erg)))
        x0 = np.reshape(x0_t, (len(x_erg),1))
        TEMP_P = self.P.toarray()
        #print("x0",np.shape(x0),type(x0))
        #print("self.P",np.shape(TEMP_P),type(TEMP_P))
        one = np.matmul(TEMP_P, x0)
        one = np.matmul(x0_t,one)

        two = np.matmul(self.q,x0_result)

        print("xPx + xq = ",one," + ",two," = ",one+two)


    def solve_for_x_regularized(self):

        #for i in range(self.input_size*self.T_f):
        #    u[self.input_size*self.T_f + self.output_size*self.T_f + i] = u[self.input_size*self.T_f + self.output_size*self.T_f + i]*-1
        
        #print("u: ",np.shape(u),u)
        """
        l = np.array([g_lower_bound])
        u = np.array([g_upper_bound])

        for i in range(1,g_len):
            l = np.vstack((l, [g_lower_bound]))
            u = np.vstack((u,  [g_upper_bound]))

        for i in range(self.T_f):
            l = np.vstack((l,  np.reshape(self.output_constrains_lb, (self.output_size,1))))
            u = np.vstack((u,  np.reshape(self.output_constrains_ub, (self.output_size,1))))

        for i in range(self.T_f):
            l = np.vstack((l,  np.reshape(self.input_constrains_lb, (self.input_size,1))))
            u = np.vstack((u,  np.reshape(self.input_constrains_ub, (self.input_size,1))))
        #l = np.flip(l)
        #u = np.flip(u)
        #print("lower bounds: ",np.shape(l),l)
        """

        prob = osqp.OSQP()
        prob.setup(self.P, self.q, self.A, self.l, self.u, alpha=0.01,max_iter= 10000,verbose = False)
        res = prob.solve()
        #print(res.x)

        #erg = opt.minimize(self.minFunction_regularized,x0,method='SLSQP',constraints = cons,options = opts,bounds =bnds)
        return res.x

    def updateReferenceWaypoint(self,new_y_r):
        if len(new_y_r) != self.output_size:
            print("Wrong size for reference point. Must be: ",self.output_size)
        self.y_r = new_y_r
        self.recalculate_g_r
        self.update_q()

    # call when changing y_r, u_r
    def recalculate_g_r(self):
        self.g_r = []
        self.data_inversed_regularized = np.vstack((self.U_f,self.Y_f))    
        self.data_inversed_regularized = np.vstack((self.Y_p,self.data_inversed_regularized))
        self.data_inversed_regularized = np.vstack((self.U_p,self.data_inversed_regularized))
        self.g_r = np.linalg.pinv(self.data_inversed_regularized)
        
        temp = []
        ur_tmep = np.array(self.u_r).T
        T_ini_ur = np.kron(np.ones((self.T_ini,1)), ur_tmep)
        T_ini_yr = np.kron(np.ones((self.T_ini,1)), self.y_r)
        T_f_ur = np.kron(np.ones((self.T_f,1)), self.u_r)
        T_f_yr = np.kron(np.ones((self.T_f,1)), self.y_r)
        temp = np.vstack((T_f_ur,T_f_yr))
        temp = np.vstack((T_ini_yr,temp))
        temp = np.vstack((T_ini_ur,temp))
        #print("ur yr stacked:",np.shape(temp.flatten()),temp.flatten())
        self.g_r = np.matmul(self.g_r,temp.flatten())
        
    
    def updateIn_Out_Measures(self,new_input_measure,new_output_measures,verbose = False):
        if(len(new_input_measure) != self.input_size or len(new_output_measures) != self.output_size):
            print("Trying to update with wrong array sizes")
        
        if verbose == True:
            print("Updating in: ",new_input_measure," out: ",new_output_measures)

        self.u_ini = np.vstack((self.u_ini,new_input_measure))
        self.y_ini = np.vstack((self.y_ini,new_output_measures))

        while(len(self.y_ini) > self.T_ini):
            self.u_ini = np.delete(self.u_ini,0,0)
            self.y_ini = np.delete(self.y_ini,0,0)

        # if y_ini is filled, only then update q, as it requires y_ini to be T_ini long
        # if this is not checked then you can not initalize y_ini with this function in the beginning
        if len(self.y_ini) == self.T_ini:
            self.update_q()

    
    def generateHankel_Collums(self,L,data_sequence):
        # this generates a hankel amtrix with the size : L x (data_length-L) as seen my each input sequence
        # viewed with each data point it is:      L * input_size  X  (data_length-L)*input_size
        # start with i:L+i -> flatten -> stick to right of H
        T = len(data_sequence)
        data_sequence = np.asarray(data_sequence)
        sequence_length = len(data_sequence[0])
        H = data_sequence[0:L,:].flatten()
        for i in range(1,T-L):
            one_collum = np.asarray(data_sequence[i:(L+i),:])
            one_collum = one_collum.flatten()
            H = np.column_stack((H, one_collum))
        #print("H:",np.shape(H)," == "(L * sequence_length, len(data_sequence) - L))
        return H

    def test_g_validity(self):
            u,y,g = self.getInputOutputPrediction()
            two = np.reshape(np.matmul(self.U_f,g),(self.T_f,self.input_size))
            three = np.reshape(np.matmul(self.Y_f,g),(self.T_f,self.output_size))
            print("u by calculation:",two)
            print("u by minimization:",u)

            print("y by calculation:",three)
            print("y by minimization:",y)