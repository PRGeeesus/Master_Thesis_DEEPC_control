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
from SimpleSystems import SimpleSystem1
import time

def main():
    system1 = SimpleSystem1(0,0,0.5)
    input = 0.5
    for i in range(1,200):
        #system1.OneTick(input)
        #if i == 50:
        #    input = -0.2
        system1.OneTick((random.random()-0.5)*2)
        #system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
    outputs = system1.SystemHistory[:,1]
    #plt.plot(outputs)
    #plt.show()

    original_data = outputs

    T_ini = 6# über 3 = schwingen, unter 3 = linearer
    T_f = 25
    ctrl = C3.Controller(system1.SystemHistory,T_ini,T_f,1,1)
    SOLLWERT = 25
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateIOConstrains([-1],[1],[-1],[1])
    ctrl.update_lambda_g(0.0000001)
    ctrl.update_lambda_s(100)
    ctrl.updateControlCost_R([[1]])
    ctrl.updateTrackingCost_Q([[1]])
    #ctrl.updateReferenceInput([0.5])
    system1.resetSystem()
    predictions_y = []
    applied_inputs = []
    soll = [SOLLWERT]
    for i in range(1,200):
        #if i == 50:
        #    ctrl.updateReferenceWaypoint([15])
        ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction(verbose = False)
        soll.append(ctrl.y_r[0])
        for j in range(1):
            predictions_y.append(y_star[j][0])
            system_output = system1.OneTick(u_star[j][0])
            applied_inputs.append(system1.u)
            ctrl.updateIn_Out_Measures(u_star[j],[system_output])
            if i%50 == 0:
                print(i,": ",u_star[j][0],system_output,y_star[j][0])
    
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

import control

syst = control.tf(1,[0.25,0.25,1])


def main3():
    #response = control.impulse_response(syst)
    times = []
    inputs = []
    for i in range(200):
        times.append(i*0.12)
        #if i > 100:    
        #    inputs.append(4)
        #else:
        #    inputs.append(0.1)
        inputs.append(random.randint(0,5))

    #response = control.forced_response(syst,times,inputs)
    time_2 = []
    outputs_2 = []
    inputs_2 = []
    input = 4
    dt = 0.1
    prev_input = 0
    T, yout_, x_out = control.initial_response(syst,[0,dt],return_x = True)
    print(T,yout_,x_out)
    time_2.append(T)
    outputs_2.append(yout_[0])
    inputs_2.append(input)
    
    for i in range(200):
        T, yout_, x_out = control.forced_response(syst,[0,dt],[prev_input,input],x_out[1],return_x = True)
        print(yout_)
        time_2.append(T)
        outputs_2.append(yout_[1])
        inputs_2.append(input)
        prev_input = input
    print(outputs_2)
    plt.plot(outputs_2,c = "r")
    #plt.plot(inputs_2,time_2,c="b")
    plt.show()

#main()
#main2()
main3()



