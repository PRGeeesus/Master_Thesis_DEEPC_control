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
