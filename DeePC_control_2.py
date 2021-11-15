
import numpy as np
import scipy.optimize as opt


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

        #self.input_constrains = [(-1.0,1.0),(-1.0,1.0),(None,None)]
        #self.output_constrains = [(-500.0,500.0),(-500.0,500.0),(-180.0,180.0)]
        self.input_constrains = [(-1000,1000),(-1000,1000),(-1000,1000)]
        self.output_constrains = [(-1000,1000),(-1000,1000),(-1000,1000)]

        # defines:
        self.u_r = np.array([0.0,0.0,0.0]) # steady state reference inout is no throttle, steer or brake
        self.y_r = np.array([])            # steady state reference output should probably be the reference waypoint I want to reach ?

        #self.Q = np.matrix([[40,0,0],[0,40,0],[0,0,40]]) #tracking error cost Martix taken from paper
        self.Q = np.asarray([[10,0,0],[0,10,0],[0,0,1000]])
        #self.R = np.matrix([[160,0,0],[0,4,0],[0,0,4]])  # quardatic control effort cost
        self.R = np.asarray([[100,0,0],[0,100,0],[0,0,100]])
        
        #to be updated as the controller runs:
        self.y_ini = np.zeros((self.output_size)) # T_ini most recent input/output measures
        self.u_ini = np.empty((self.input_size))
        self.input_sequence = self.data[:,:self.input_size] # in Test
        self.output_sequence = self.data[:,self.input_size:self.output_size + self.input_size]# in Test

        self.U_p_f = self.generateHankel_Collums(self.L,self.input_sequence)
        self.U_p = self.U_p_f[:self.T_ini*self.input_size,:] 
        self.U_f = self.U_p_f[self.L*self.input_size-self.T_f*self.input_size:,:]

        self.Y_p_f = self.generateHankel_Collums(self.L,self.output_sequence)
        self.Y_p = self.Y_p_f[:self.T_ini*self.output_size,:] 
        self.Y_f = self.Y_p_f[self.L*self.output_size-self.T_f*self.output_size:,:]

        # these are only needed for Regularization:
        self.g_r = []
        self.data_inversed_regularized = np.vstack((self.U_f,self.Y_f))    
        self.data_inversed_regularized = np.vstack((self.Y_p,self.data_inversed_regularized))
        self.data_inversed_regularized = np.vstack((self.U_p,self.data_inversed_regularized))
        self.data_inversed_regularized = np.linalg.pinv(self.data_inversed_regularized,rcond=1e-15)

        self.lambda_s = 1.0
        self.lambda_g = 1.0
    
    def getInputOutputPrediction(self):
        erg = self.solve_for_u_y_g_unregularized()
        x0_result = erg.x
        g = x0_result[self.T_f*self.input_size + self.T_f*self.output_size : ]

        u = x0_result[:self.T_f*self.input_size]
        u_star = np.reshape(np.matmul(self.U_f,g),(self.T_f,self.input_size))
        
        y =  x0_result[self.T_f*self.input_size : self.T_f*self.input_size + self.T_f*self.output_size]
        y_star = np.reshape(np.matmul(self.Y_f,g),(self.T_f,self.output_size))

        u = np.reshape(u, (self.T_f,self.input_size))
        u_star = np.reshape(u_star, (self.T_f,self.input_size))
        y = np.reshape(y, (self.T_f,self.output_size))
        y_star = np.reshape(y_star, (self.T_f,self.output_size))

        return u,y,u_star,y_star,g

    def test_g_validity(self):
        u,y,g = self.getInputOutputPrediction()
        two = np.reshape(np.matmul(self.U_f,g),(self.T_f,self.input_size))
        three = np.reshape(np.matmul(self.Y_f,g),(self.T_f,self.output_size))
        print("u by calculation:",two)
        print("u by minimization:",u)

        print("y by calculation:",three)
        print("y by minimization:",y)

    def solve_for_u_y_g_unregularized(self):
        init_value = 0.5
        
        u_len = self.T_f*self.input_size
        y_len =  self.T_f*self.output_size
        g_len = len(self.input_sequence) - self.L

        x0 = np.full(((u_len + y_len + g_len)),init_value)
        # I assume x0 as [u11,u12,u13,u21,u22,u23,...,y11,12,y13,y21,...g1,g2,g3,...,g_T-L]
        bnds = []
        for i in range(self.T_f):
            for i in self.input_constrains:
                bnds.append(i)

        for i in range(self.T_f):
            for i in self.output_constrains:
                bnds.append(i)
        
        for i in range(g_len):
            bnds.append((None,None))
        #print(len(bnds)," - ",np.shape(bnds)," - ",len(x0),bnds)
        #cons = ({'type': 'eq', 'fun': self.constrain1},
        #        {'type': 'eq', 'fun': self.constrain2})
        cons = ({'type': 'eq', 'fun': self.constrain1})

        opts ={'maxiter': 1000, 'ftol': 0.0000001, 'iprint': 1, 'disp': True, 'eps': 0.00000001}
        erg = opt.minimize(self.minFunction,x0,method='SLSQP',constraints = cons,options = opts,bounds =bnds)
        return erg
    
    def solve_for_u_y_g_regularized(self):
        init_value = 0.5
        
        u_len = self.T_f*self.input_size
        y_len =  self.T_f*self.output_size
        g_len = len(self.input_sequence) - self.L

        x0 = np.full(((u_len + y_len + g_len)),init_value)
        # I assume x0 as [u11,u12,u13,u21,u22,u23,...,y11,12,y13,y21,...g1,g2,g3,...,g_T-L]
        bnds = []
        for i in range(self.T_f):
            for i in self.input_constrains:
                bnds.append(i)

        for i in range(self.T_f):
            for i in self.output_constrains:
                bnds.append(i)
        
        for i in range(g_len):
            bnds.append((None,None))
        #print(len(bnds)," - ",np.shape(bnds)," - ",len(x0),bnds)
        #cons = ({'type': 'eq', 'fun': self.constrain1},
        #        {'type': 'eq', 'fun': self.constrain2})
        cons = ({'type': 'eq', 'fun': self.constrain1})

        opts ={'maxiter': 1000, 'ftol': 0.0000001, 'iprint': 1, 'disp': True, 'eps': 0.00000001}
        erg = opt.minimize(self.minFunction_regularized,x0,method='SLSQP',constraints = cons,options = opts,bounds =bnds)
        return erg

    def constrain1(self,x0):
        
        g = x0[self.T_f*self.input_size + self.T_f*self.output_size :]
        #print(np.shape(A)," - ",np.shape(g)," - ",np.shape(a))
        ans = np.matmul(self.U_p,g) - self.u_ini.flatten()
        ans2 = np.matmul(self.Y_p,g) - self.y_ini.flatten()
        #ans = np.matmul(A,g) - a
        return ans + ans2

    def constrain2(self,x0):
        g = x0[self.T_f*self.input_size + self.T_f*self.output_size :]
        ans = np.matmul(self.Y_p,g) - self.y_ini.flatten()
        return ans
    
    def minFunction(self,x0):
        u = x0[:self.T_f*self.input_size]
        u = np.reshape(u, (self.T_f,self.input_size))
        y = x0[self.T_f*self.input_size : self.T_f*self.input_size + self.T_f*self.output_size]
        y = np.reshape(y, (self.T_f,self.output_size))
        #g = x0[self.T_f*self.input_size + self.T_f*self.output_size :]
        summation = 0
        for i in range(self.T_f):
            summation = summation + self.costFunction(u[i],y[i])
        
        return summation

    def minFunction_regularized(self,x0):
        u = x0[:self.T_f*self.input_size]
        u = np.reshape(u, (self.T_f,self.input_size))
        y = x0[self.T_f*self.input_size : self.T_f*self.input_size + self.T_f*self.output_size]
        y = np.reshape(y, (self.T_f,self.output_size))

        g = x0[self.T_f*self.input_size + self.T_f*self.output_size :]

        second_term = np.matmul(self.Y_p,g) - self.y_ini
        second_term = np.linalg.norm(second_term,2)
        second_term = np.square(second_term) * self.lambda_s
        #g = x0[self.T_f*self.input_size + self.T_f*self.output_size :]
        summation = 0
        for i in range(self.T_f):
            summation = summation + self.costFunction(u[i],y[i])
        
        return summation
    def r_of_g(self,g):
        print(np.shape(self.g))
        print(np.shape(self.g_r))

    def costFunction(self,u,y):
        
        deltay = y - self.y_r
        deltau = u - self.u_r

        temp_y = np.matmul(self.Q,deltay)
        temp_y = np.matmul(deltay,temp_y)
        temp_u = np.matmul(self.R,deltau)
        temp_u = np.matmul(deltau,temp_u)

        return temp_y + temp_u


    def updateReferenceWaypoint(self,new_y_r):
        if len(new_y_r) != self.output_size:
            print("Wrong size for reference point. Must be: ",self.output_size)
        self.y_r = new_y_r
        
        self.g_r = np.reshape(self.y_r,(self.output_size,1))
       # print(self.g_r)
        #self.g_r = np.vstack((self.g_r,np.reshape(self.u_r,(self.input_size,1))))
        #print(self.g_r)
        for i in range(1,self.T_f):
            self.g_r = np.vstack((self.g_r,np.reshape(self.y_r,(self.output_size,1))))
        for i in range(self.T_f):
            self.g_r = np.vstack((self.g_r,np.reshape(self.u_r,(self.input_size,1))))
        for i in range(self.T_ini):
            self.g_r = np.vstack((self.g_r,np.reshape(self.y_r,(self.input_size,1))))
        for i in range(self.T_ini):
            self.g_r = np.vstack((self.g_r,np.reshape(self.u_r,(self.output_size,1))))
        print(np.shape(self.g_r))
         
        
    
    def updateIn_Out_Measures(self,new_input_measure,new_output_measures):
        if(len(new_input_measure) != self.input_size or len(new_output_measures) != self.output_size):
            print("Trying to update with wrong array sizes")
        
        self.u_ini = np.vstack((self.u_ini,new_input_measure))
        self.y_ini = np.vstack((self.y_ini,new_output_measures))

        while(len(self.y_ini) > self.T_ini):
            self.u_ini = np.delete(self.u_ini,0,0)
            self.y_ini = np.delete(self.y_ini,0,0)
        
        

    
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
