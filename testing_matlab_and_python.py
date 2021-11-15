
import matlab.engine
import matlab
import numpy as np


class MatlabController():
    def __init__(self,data,y_reference):
        self.env = matlab.engine.connect_matlab()
        self.env.workspace["data"] = data
        self.env.workspace["y_reference"] = y_reference

        # self.env.workspace["matlab_var"]
        # self.env.library(5.0) # call a function called library
    def init(self):
        pass
    def getOptimalControlSequence(self):
        pass
    
    def updateInputOutputMeasures(self):
        pass

    def updateReferenceWaypoint(self):
        pass
"""
print("Trying connectio to matlab")
eng = matlab.engine.connect_matlab()
x = eng.workspace["matlab_var"]
print(np.sqrt(x))
# nargout=0 specifies no return value, but sets output in matlab doc
print(eng.library(5.0)) # call a function

eng.workspace["matlab_var"] = 3.0
eng.workspace["crated_from_python"] = [100,99,88,77]
"""