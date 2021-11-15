from math import sin
import numpy as np
from numpy.lib.function_base import sinc
import scipy.optimize as opt


def BoxProblem():
    # Problem: Sheet with Width * Length hgas cutours at corners of size x
    # What is the maximum Volume of a folded Box ou of that paper for x?

    # Solution: Volume ~ 10.56 at x ~ 3.96

    x0 = [1.0] # inital Guesss

    # options
    opts ={'maxiter': 100, 'ftol': 0.00001, 'iprint': 1, 'disp': True, 'eps': 0.00000001}

    # bounds
    bnds = [(0, 10)]

    def minimizationFunction(parameters):
        x = parameters[0]
        print(x)
        width = 30
        length = 20
        volume = (width - 2*x)*(length - 2*x)*x
        return -volume

    erg = opt.minimize(minimizationFunction,x0,method='SLSQP',options = opts,bounds =bnds)
    print(erg)
    print(minimizationFunction(erg.x))

# this can not search for the global minimum (0,0) but once it its convex surrounding
# contains the global minimum, it has no problem:
# inital guess -> (8,0) -> no global minimum
# inital guess -> (2,2) -> global minimum
def SincOptimization():
    # Problem: Find the minimum of z = 2* -Sinc(x)*sin(y)
    # What is the minimal point of the plane?

    # Solution: (x,y) = (0,0) = -2

    x0 = [(2.0,2.01)] # inital Guesss

    # options
    opts ={'maxiter': 100, 'ftol': 0.001, 'iprint': 1, 'disp': True, 'eps': 0.00000001}

    # bounds
    bnds = [(None, None),(None,None)]

    def minimizationFunction(parameters):
        x = parameters[0]
        y = parameters[1]
        print(parameters)
        factor = 2
        Z = -factor * np.abs((np.sin(x)/x) * (np.sin(y)/y))
        #Z = factor*((-np.sinc(x)) * np.sinc(y))
        return Z

    erg = opt.minimize(minimizationFunction,x0,method='SLSQP',options = opts,bounds =bnds)
    print(erg)
    print(minimizationFunction(erg.x))

def findMinofX2():
    x0 = [5.0,5.0] # inital Guesss

    # options
    opts ={'maxiter': 1000, 'ftol': 0.001, 'iprint': 1, 'disp': True, 'eps': 0.00000001}

    # bounds
    bnds = [(0, None),(0,None)]

    def minimizationFunction(parameters):
        x1 = parameters[0]
        x2 = parameters[1]
        x = [x1,x2]
        temp = np.matmul([[1,0],[0,1]],x)
        temp = np.matmul(x,temp) + 1

        erg = temp + np.matmul([1,0],x)

        return erg
    def constrain1(x0):
        x1 = x0[0]
        x2 = x0[1]
        return 3*x1 - x2 + 4
    
    def constrain2(x0):
        x2 = x0[1]
        return 2*x2 -1 

    #cons = ({'type': 'eq', 'fun': constrain1},
    #        {'type': 'eq', 'fun': constrain2})
    cons = ({'type': 'eq', 'fun': constrain1})

    erg = opt.minimize(minimizationFunction,x0,method='SLSQP',constraints = cons,options = opts,bounds =bnds)
    print(erg)
    print(minimizationFunction(erg.x))


findMinofX2()