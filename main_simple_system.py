# goal is to use DEEPC to predict a simple integrator:
# x(k+1) = x(k) + I*u(k)
# step1: y(0) = x(0)
#        x(1) = y(0) + T * u(0)
#setp2: y(1) = x(1) + T * u(1)
#       x(2) = y(1)
#setpk  y(k) = x(k) + T* u(k)
#       x(k +1) = y(k)
import matplotlib.pyplot as plt
import random
import DeePC_OSQP as C3
import numpy as np

class SimpleSystem:
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
        inputt = self.truncateInput(input)
        #inputt = input
        self.u = inputt
        self.y = self.x + self.T * inputt
        self.x = self.y
        self.SystemHistory = np.vstack((self.SystemHistory,[inputt,self.y]))
        return self.y

def main():
    system1 = SimpleSystem(0,0,0.5)
    input = 0.1
    for i in range(1,200):
        system1.OneTick((random.random()-0.5)*2)
        #system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
    outputs = system1.SystemHistory[:,1]
    #plt.plot(outputs)
    #plt.show()

    original_data = outputs

    T_ini = 2 # über 3 = schwingen, unter 3 = linearer
    T_f = 25
    ctrl = C3.Controller(system1.SystemHistory,T_ini,T_f,1,1)
    SOLLWERT = 25
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateIOConstrains([-1],[1],[0],[0])
    #ctrl.update_lambda_g(500)
    #ctrl.update_lambda_s(1)
    #ctrl.updateControlCost_R([[100]])
    #ctrl.updateTrackingCost_Q([[100]])
    #ctrl.updateReferenceInput([0.5])
    system1.resetSystem()
    predictions_y = []
    applied_inputs = []
    soll = [SOLLWERT]
    for i in range(200):
        #if i == 50:
        #    ctrl.updateReferenceWaypoint([20])
        
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        
        soll.append(ctrl.y_r[0])
        for i in range(1):
            predictions_y.append(y_star[i][0])
            system_output = system1.OneTick(u_star[i][0])
            applied_inputs.append(system1.u)
            print(":",u_star[i][0],system_output,y_star[i][0])
            ctrl.updateIn_Out_Measures(u_star[i],[system_output])
    
    outputs2 = system1.SystemHistory[:,1]
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])

    fig, ax1 = plt.subplots()
    titlesting = "T_ini:" + str(T_ini) + " T_f:" + str(T_f) +" lg:" + str(ctrl.lambda_g) + " ls:"+ str(ctrl.lambda_s)
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.plot(original_data,label='Init Data')
    ax1.plot(soll,label='SOLLWERT',c="y")
    ax1.plot(predictions_y,label='predictions',c="r")
    ax1.plot(outputs2,label="system behaviour",c="g")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(applied_inputs,label="applied inputs",c="b")
    ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()


from simple_pid import PID
def main2():
    SOLLWERT = 10

    pid = PID(0.3, 0.1, 0.00, setpoint=SOLLWERT)
    pid.output_limits = (-1.0, 1.0)
    pid.sample_time = 0.00
    system1 = SimpleSystem(0.0,0.0,0.5)
    system1.resetSystem()
    applied_inputs = []
    system_output = system1.y
    for i in range(200):
        #if i == 22:
        #    ctrl.updateReferenceWaypoint([20])
        
       
        control = pid(system_output)
        system_output = system1.OneTick(control)
        applied_inputs.append(control)
        print(":",control,system_output)
    
    outputs2 = system1.SystemHistory[:,1]
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])

    fig, ax1 = plt.subplots()
    titlesting = "Simple PID COntrol"
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.plot([SOLLWERT for i in range(len(outputs2))],label='SOLLWERT',c="y")
    ax1.plot(outputs2,label="system behaviour",c="g")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(applied_inputs,label="applied inputs",c="b")
    ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()



main()
#main2()



