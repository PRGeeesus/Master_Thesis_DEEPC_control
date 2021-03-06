
from operator import matmul
from random import triangular
import numpy as np
from numpy.core.fromnumeric import size
import scipy.optimize as opt

import sys

from scipy.sparse import data

class Controller:
    # controller has to be initalized with data:
    def __init__(self,data,T_ini,T_f,input_size,output_size):
        self.sample_size = len(data)
        self.data = data
        self.input_sequence = []
        self.output_sequence = []
        self.T_ini = T_ini
        self.T_f = T_f

        self.input_size = input_size
        self.output_size = output_size

        # defines:
        self.u_r = np.array([0.0,0.0,0.0]) # steady state reference inout is no throttle, steer or brake
        self.y_r = np.array([])            # steady state reference output should probably be the reference waypoint I want to reach ?

        #self.Q = np.matrix([[40,0,0],[0,40,0],[0,0,40]]) #tracking error cost Martix taken from paper
        self.Q = np.matrix([[4,0,0],[0,4,0],[0,0,4]])
        #self.R = np.matrix([[160,0,0],[0,4,0],[0,0,4]])  # quardatic control effort cost
        self.R = np.matrix([[40,0,0],[0,4,0],[0,0,1]])
        
        #to be updated as the controller runs:
        self.y_ini = [] # T_ini most recent input/output measures
        self.u_ini = []

        # initalization gives:
        #self.Y_p_f 
        #self.Y_p = self.Y_p_f[:self.T_ini]             
        #self.Y_f = self.Y_p_f[L- self.T_f:]  
        #self.U_p_f
        #self.U_p = self.U_p_f[:self.T_ini] 
        #self.U_f = self.U_p_f[L-self.T_f:]

        #for regularization:
        #self.lambda_s = 0.000000075 # 7.5 x 10^-8 taken from paper
        self.lambda_s = 1 # 7.5 x 10^-8 taken from paper
        self.q_norm = 2
        #self.lambda_g = 500
        self.lambda_g = 1

        self.init() # done to the initalization matricies up from the data proveided

    def getOptimalControlSequence(self):
       
        y_optimal, u_optimal, g_star = self.find_g_star()

        # u* = U_f x g*
        #print(np.asarray(self.U_f).shape)
        #print(np.asarray(g_star).shape)
        u_star = np.matmul(self.U_f,g_star)
        #print(np.asarray(u_star).shape)
        y_star = np.matmul(self.Y_f,g_star)
        # group the optimal outputs into 
        optimal_inputs = []
        predicted_output = []
        for i in range(int((len(u_star)/self.input_size)-1) ):
            optimal_inputs.append(u_star[i*3:(i+1)*3])
            predicted_output.append(y_star[i*3:(i+1)*3])

        print("Test: u* = ",u_star," u =",u_optimal," u* - u:",(u_star[0]-u_optimal))
        return optimal_inputs,predicted_output

    def find_g_star(self):
        # define system
        # scipy.optimize.minimize(fun, x0, args=(), method=None, jac=None, hess=None, hessp=None, bounds=None, constraints=(), tol=None, callback=None, options=None)
        # opt.minimize()
        # constraints = ()
        # fun = funtion = Regularized DeePC for nonlinear noisy systems (8)
        # fun needs y,u,g
        # fun(x, *args) -> float x = n dimensional array
        init_value = 0.3
        g_width = self.sample_size-(self.T_ini + self.T_f) # maybe +1 here
        # g is member of R ^ Td-T_ini -T_f # seee abobe (6)
        # u_ini is R ^ T_ini X inputsize
        # u is R ^ Tf x input size
        # y is R ^ Tf x output size
        x0 = np.full(((g_width + self.T_f*self.input_size + self.T_f*self.output_size)),init_value)
        #print("Dim. of g = ",g_width, " -> should be: ", len(self.data)-(self.T_ini + self.T_f))
        #print("Dim of x0: ",np.shape(x0), " ==> ", g_width, " + ", self.T_ini,"*",self.input_size," + ",self.T_f,"*",self.output_size," ")
        print("x0 shape:",np.shape(x0))                             
        opts ={'maxiter': 100, 'ftol': 0.00001, 'iprint': 1, 'disp': True, 'eps': 0.000001}
        #opts ={'maxiter': 1000, 'xtol': 0.00001, 'disp': True}
        
        #output_bounds = [(None, None), (None, None),(-180, 180)]
        #input_bounds = [(0, 0.2),(-1.0, 1.0),(0, 1.0)] 
        # (xmin, xmax) (ymin,ymax)   (yaw min/max)  (throttle) (steer)       (brake)
        bnds = []
        # fill bounds for u
        for i in range(self.T_f):
            bnds.append((None, None))
            bnds.append((None, None))
            bnds.append((-180, 180))
        # fill bounds for y
        for i in range(self.T_f):
            bnds.append((0, 0.2))
            bnds.append((-1.0, 1.0))
            bnds.append((0, 1.0))
        #bounds for g -> no idea so just leave none
        for i in range(g_width): bnds.append((None, None))
        print("bounds shape: ",np.shape(bnds))
        
        # constrains:
        ub = lb = np.zeros(self.T_ini*self.input_size + self.T_f*self.input_size + self.T_f*self.output_size )
        print("constrains shape:" ,np.shape(ub))
        nlc = opt.NonlinearConstraint(self.nlconstrains, ub, lb)
        '''cons = ({'type': 'eq', 'fun': self.constrain1},
                {'type': 'eq', 'fun': self.constrain2},
                {'type': 'eq', 'fun': self.constrain3})'''

        cons = ({'type': 'eq', 'fun': self.constrain1})

        # this solves but does not use constraints
        #erg = opt.minimize(self.minimizationFunction,x0,method='SLSQP',options = opts,bounds =bnds)

        erg = opt.minimize(self.minimizationFunction,x0,method='SLSQP',constraints = cons,options = opts,bounds =bnds)
        #erg = opt.minimize(self.minimizationFunction,x0,method='SLSQP',constraints = nlc,options = opts,bounds =bnds)
        #erg = opt.minimize(self.minimizationFunction,x0,method='trust-constr',options = opts,bounds =bnds,constraints=cons)
        #print(erg)
        if erg.success == True:
            solution = erg.x
            y_uptimal = solution[:self.input_size*self.T_f]
            u_uptimal = solution[self.input_size*self.T_f : self.input_size*self.T_f + self.output_size*self.T_f]
            #print("What are these: ",y_uptimal,u_uptimal)
            g_star = solution[self.input_size*self.T_f + self.output_size*self.T_f :]
        else:
            print("Error, Optimization could not be solved:")
            print(erg)
            y_uptimal = u_uptimal = g_star = None

        return y_uptimal,u_uptimal,g_star

    # U_p*g - u_ini = 0
    def constrain1(self,x0):
        g = x0[self.input_size*self.T_f + self.output_size*self.T_f : ]
        u_ini = np.reshape(self.u_ini,(self.T_ini*self.input_size))
        return np.matmul(self.U_p,g) - u_ini
    # U_f*g - u = 0
    def constrain2(self,x0):
        g = x0[self.input_size*self.T_f + self.output_size*self.T_f : ]
        u = x0[self.output_size*self.T_f : self.input_size*self.T_f + self.output_size*self.T_f]
        return np.matmul(self.U_f,g) - u
        
    # Y_f*g - y = 0
    def constrain3(self,x0):
        g = x0[self.input_size*self.T_f + self.output_size*self.T_f : ]
        y = x0[0:self.output_size*self.T_f]
        return np.matmul(self.Y_f,g) - y

    def nlconstrains(self,x0):
        y = x0[0:self.output_size*self.T_f]
        #y = np.reshape(y,(self.T_f,self.output_size))
        u = x0[self.output_size*self.T_f : self.input_size*self.T_f + self.output_size*self.T_f]
        #u = np.reshape(u,(self.T_f,self.input_size))
        g = x0[self.input_size*self.T_f + self.output_size*self.T_f : ]

        #print("self.U_p", np.shape(self.U_p))
        #print("self.U_f", np.shape(self.U_f))
        #print("self.Y_f", np.shape(self.Y_f))
        data_vector =np.vstack((self.U_p, self.U_f,self.Y_f))
        #print("[U_p U_f Y_f] shape = ",np.shape(data_vector))

        #print("g shape:",np.shape(g))
        u_ini = np.reshape(self.u_ini,(self.T_ini*self.input_size))
        #print("u_ini",np.shape(u_ini),u_ini)
        #print("u shape: ",np.shape(u),u)
        #print(" y shape: ",np.shape(y),y)
        a_vector = np.hstack((u_ini,y,u))
        a_vector = a_vector[:,None].flatten()
        #print(" a_vector",np.shape(a_vector))
        ans = np.matmul(data_vector,g) - a_vector
        print(ans)
        return ans
    # this is what is written in the summation in equation (8)
    # input should include y,u, y_s, Y_p, g, y_ini
    # minimization parameters: parameters = [u,y,g]
    # u = 3 y = 3, g = 9 so 15 in total?
    def minimizationFunction(self,parameters):
        # parameters are: [y1,y2,y3 , u1,u2,u3, g1,g2,g3, ..., g(self.sample_size-(self.T_ini + self.T_f))]
        y = parameters[0:self.output_size*self.T_f]
        y = np.reshape(y,(self.T_f,self.output_size))
        u = parameters[self.output_size*self.T_f : self.input_size*self.T_f + self.output_size*self.T_f]
        u = np.reshape(u,(self.T_f,self.input_size))
        g = parameters[self.input_size*self.T_f + self.output_size*self.T_f: ]
        y_ini = self.y_ini.flatten()
        sum_term = 0

        second_term = self.lambda_s*np.linalg.norm((np.matmul(self.Y_p,g) - y_ini),self.q_norm)**2
        third_term = self.regularize_g(g)
        for i in range(self.T_f):
            cost = self.costfunction(y[i],u[i])
            sum_term = cost + second_term + third_term
        #print(cost,second_term,third_term)
        return sum_term
    
    def costfunction(self,y,u):
        # implementation of (9) for (8)
        # not shure but I think i need to give the reference waypoint here,
        # as the cost is the difference between the output and the desired tracking of the waypoint
        
        # not shure why i put this in ???
        #if self.y_r.all() != np.array(self.reference_waypoint).all():
        #    self.updateReferenceWaypoint(self.reference_waypoint)
        
        # self.y_r = reference waypoint
        
        y_diff = np.asarray(y) - np.asarray(self.y_r) 
        u_diff = np.asarray(u) - np.asarray(self.u_r)
        
        temp  = np.matmul(y_diff,self.Q)
        output_cost = np.matmul(temp,y_diff)

        temp2 = np.matmul(u_diff.T,self.R)
        input_cost = np.matmul(temp2,u_diff)

        cost = output_cost + input_cost
        return cost

    def updateInputOutputMeasures(self,inputs,outputs):
        if len(inputs) != self.input_size or len(outputs) != self.output_size:
            print("Tried to update wrongly sized in/outputs in updateInputOutputMeasures()")
        if len( self.u_ini) == 0:
             self.u_ini = np.zeros(self.input_size)
        if len( self.y_ini) == 0:
             self.y_ini = np.zeros(self.output_size)

        npa_inputs = np.array(inputs)
        npa_outputs = np.array(outputs)
        self.u_ini = np.vstack([self.u_ini, npa_inputs])
        self.y_ini = np.vstack([self.y_ini, npa_outputs])
        
        #shifting so that uini and y ini are always T_ini x input/output size
        while(np.shape(self.u_ini)[0] > self.T_ini):
            self.u_ini = np.delete(self.u_ini, (0), axis=0)
        
        while(np.shape(self.y_ini)[0] > self.T_ini):
            self.y_ini = np.delete(self.y_ini, (0), axis=0)




    # returns None if sequence of signals is not long enough or L was chosen too large
    def generateHankel(self,L,innput):
        # L is like the window size that is sliding ofer the inputs
        #! it has to follw: T >= (m+1)*L -1
        T = len(innput)
        if (T < (len(innput[0])+1)*L -1):
            print(T," < ",(len(innput[0])+1)*L -1," Invalid sizes of T_ini and T_f, or the dataset is too small")
            return None
        # [ [collum0],[collum1],[collum2], ... ,[collum T-L] ]
        # collum 0 = 
        H_matr = []
        for i in range(T-L):
            
            list_of_sequence = innput[i:L + i]
            collum = []
            for one_list in list_of_sequence:
                for element in one_list:
                    collum.append(element)
            H_matr.append(collum)
        # entries are the collums
        # it is now as:
        #[[  0   0   0   0   0   0   1   1   1  -1  -1  -1   2   2   2  -2  -2  -2
        #3   3   3  -3  -3  -3   4   4   4  -4  -4  -4   5   5   5  -5  -5  -5
        #6   6   6  -6  -6  -6   7   7   7  -7  -7  -7   8   8   8  -8  -8  -8
        #9   9   9  -9  -9  -9  10  10  10 -10 -10 -10]
        #[  1   1   1  -1  -1  -1   2   2   2  -2  -2  -2   3   3   3  -3  -3  -3
        #  4   4   4  -4  -4  -4   5   5   5  -5  -5  -5   6   6   6  -6  -6  -6
        # 7   7   7  -7  -7  -7   8   8   8  -8  -8  -8   9   9   9  -9  -9  -9
        #10  10  10 -10 -10 -10  11  11  11 -11 -11 -11]

        # entries are the rows:
       #[[  0   1   2   3   4   5   6   7   8]
        #[  0   1   2   3   4   5   6   7   8]
        #[  0   1   2   3   4   5   6   7   8]
        #[  0  -1  -2  -3  -4  -5  -6  -7  -8]
        H_matr = np.asarray(H_matr).T

        #print("Hankel:") 
        #print(np.asarray(H_matr))
        #print("should be dimension: ",L*len(input[0])," X ",T-L)
        #print("is dimentsion:",np.asarray(H_matr).T.shape)
        #print(np.asarray(H_matr).T)
        return H_matr

    def updateReferenceWaypoint(self,reference_waypoint):
        self.y_r = np.asarray(reference_waypoint)


    def regularize_g(self,g):
        if self.y_r == []:
            print("Reference output not defined. Where do you even want to go?")
        g_reg = self.get_g_r(g)
        r = self.lambda_g * np.linalg.norm((g-g_reg),self.q_norm)
        return r

    def checkReferencepath(self):
        if self.y_r == []:
            print("Reference output not defined. Where do you even want to go?")

    # from function (10). g regularized
    def get_g_r(self,g):
        # TODO: rewrite so that independent of dimension of inputs. so regardless if the reference is [0,0,0] or [0,0,0,0]
        # it then should all just be stacked
        # g should be dimension (T_ini*2 + T_f*2) x (1) x (nr_of_inputs) 
        # 

        self.checkReferencepath()

        # [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
        u_r_tini = [ [j] for j in self.u_r for i in range(self.T_ini)] # this should maybe assume u_r is a collum vecor?
        #print(u_r_tini)
        #print(np.asarray(u_r_tini).shape)
        
        y_r_tini = [ [j] for j in self.y_r for i in range(self.T_ini)]

        u_r_tf = [ [j] for j in self.u_r for i in range(self.T_f)]
        y_r_tf = [ [j] for j in self.y_r for i in range(self.T_f)]

        # self.U_p is L = T_ini+T_f tall (6) and wide however many it takes to fill T-L many
        # to in this case 6 x 68 x 3
        temp1 = np.concatenate((np.asarray(self.U_p),np.asarray(self.Y_p),np.asarray(self.U_f),np.asarray(self.Y_f)))
        temp2 = np.concatenate((np.asarray(u_r_tini),np.asarray(y_r_tini),np.asarray(u_r_tf),np.asarray(y_r_tf)))
        
        #print("Start:")
        #print("Up + Yp + Uf + Yf:")
        #print(np.asarray(temp1))
        
        #print("ur + freinds stacked Tini times")
        #print(np.asarray(temp2))
        # should be fine up to here
        #print(np.asarray(temp1).shape)
        temp1 = np.linalg.pinv(temp1, rcond=1e-15, hermitian=False)
        #print(np.asarray(temp1))
        #print(np.asarray(temp1).shape)
        #print(np.asarray(temp2).shape)
        erg = np.matmul(temp1,temp2)
        #print(erg)
        #print(np.asarray(erg).shape)

        # assuming all is correct up to here:
    
        return erg
    def TestFunction(self):
        np.set_printoptions(threshold=sys.maxsize)
        #print("Data shape: ", np.shape(self.data))
        #print(" --- Output Data shape:",np.shape(self.output_sequence))
        #print(" ---  Input Data shape:",np.shape(self.input_sequence))

        #print(" ---- Hankel Matrix out of Output data:")
        #print(self.Y_p_f)
        print("It has dimension: ",np.shape(self.Y_p_f)," - should be ",self.L * len(self.output_sequence[0]) ," X ",len(self.output_sequence)-self.L)
        print(" -- The first ",self.T_ini," samples from \"Output data\" that is self.Y_p:")
        print(np.shape(self.Y_p))
        #print(self.Y_p)
        print(" -- The Last ", self.T_f, " samples from \"Output data\" that is self.Y_f:")
        print(np.shape(self.Y_f))
        #print(self.Y_f)
        testinput = self.input_sequence[:12]
        testoutput = self.output_sequence[:12]
        print("testinput: ",np.shape(testinput))
        print("testoutput: ",np.shape(testoutput))
        print("Testing: updateInputOutputMeasures")
        for i in range(len(testinput)):
            self.updateInputOutputMeasures(testinput[i],testoutput[i])
        print("updated u_ini shape:",np.shape(self.u_ini))
        print("updated y_ini shape:",np.shape(self.y_ini))

        reference = [5,-70,0]
        print("Testing: updateReferenceWaypoint(",reference,")")
        self.updateReferenceWaypoint(reference)
        print(self.y_r)

        print("Finding g*:")
        #g_width = self.sample_size-(self.T_ini + self.T_f) # maybe +1 here, im confused :/
        #x0 = [0.1 for i in range(g_width + self.input_size + self.output_size)]
        #print(np.shape(np.asarray(self.U_p))," X ",np.shape(np.asarray(x0[6:])), " - ",np.shape(np.asarray(self.u_ini)))
        #np.matmul(np.asarray(self.U_p),np.asarray(x0[6:])) - self.u_ini}
        y_out,u_out,g_star = self.find_g_star()
        print("g_star: {g_star} dim: ",np.shape(g_star),"")
        print("u_out: {u_out} dim: ",np.shape(u_out),"")
        print("y_out: {y_out} dim: ",np.shape(y_out),"")


    def init(self):
        #[car x, car y, yaw , throttle, steer, brake]
        #[output, output, output, input,input,input]
        # split that up
        # into u_d and y_d of length T_d
        for i in self.data:
            self.output_sequence.append(i[:self.output_size])
            self.input_sequence.append(i[self.output_size:])
       
        # variable for dimensions
        self.L = self.T_ini + self.T_f #is passed upon initalization
        

        self.Y_p_f = self.generateHankel(self.L,self.output_sequence)
        
        self.Y_p = self.Y_p_f[:self.T_ini*self.output_size]                           # first T_ini blocks
        self.Y_f = self.Y_p_f[self.L * self.output_size - self.T_f * self.output_size:]    # Last T_f blocks
        #print("init:")
        #print(np.asarray(self.Y_p)) # should be 6 x 19 x 3 -> confirmed
        #print("-------------------------------------")
        #print(np.asarray(self.Y_f)) # should be 25 x 19 x 3 -> confirmed

        self.U_p_f = self.generateHankel(self.L,self.input_sequence)
        
        self.U_p = self.U_p_f[:self.T_ini * self.input_size]                    #first T_ini blocks
        self.U_f = self.U_p_f[self.L *self.input_size - self.T_f * self.input_size:] # last T_f blocks
        #print(np.asarray(self.U_p))
        #print(np.asarray(self.U_f))

        #print(np.array(Y_p_f))

        #get inputs from data

        #get outputs from data

        # construct input Hankel Matrix

        # construct output Hankel Matrix

        # construct t_ini most recent past input/output measurements

