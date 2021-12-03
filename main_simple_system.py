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
<<<<<<< Updated upstream
from SimpleSystems import CessnaSystem, ChessnaMPCController, InvertedPendulimSS, SimpleSystem1,SecondOrderSystem
=======
from SimpleSystems import SimpleSystem1,SimpleSystem2
>>>>>>> Stashed changes
import time
#import control


def main():
    system1 = SimpleSystem2(0,0,0.5)
    input = 0.5
    for i in range(1,100):
        system1.OneTick(input)
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
    ctrl.updateIOConstrains([-1],[1],[-100],[100])
    ctrl.update_lambda_g(5)
    ctrl.update_lambda_s(0.5)
    ctrl.updateControlCost_R([[1]])
    ctrl.updateTrackingCost_Q([[1]])
    #ctrl.updateReferenceInput([0.5])
    system1.resetSystem()
    predictions_y = []
    applied_inputs = []
    soll = [SOLLWERT]
    for i in range(1,200):
        if i == 50:
            ctrl.updateReferenceWaypoint([15])
        if i == 150:
            ctrl.updateReferenceWaypoint([25])
        #ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
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

<<<<<<< Updated upstream
def main3():
    sys = SecondOrderSystem(250,100)
    outputs = []
    input = 30
    for i in range(1000):
        if i == 200:
            input = 15
        outputs.append(sys(input))
    fig, ax1 = plt.subplots()
    titlesting = "Simple PID COntrol"
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.plot(outputs,label="system outputs",c="g")
    plt.legend(loc=8)
    # ...
    #ax2 = ax1.twinx()
    #ax2.set_ylabel("Inputs")
    #ax2.plot(inputs,label="applied inputs",c="b")
    #ax2.plot(out_angle,label="system out_angle",c="r")
    #ax2.set_ylim([-2, 2])

    #plt.legend(loc=4)
    plt.show()
=======

#syst = control.tf(1,[0.25,0.25,1])
>>>>>>> Stashed changes

def main4():

    input = -1.0*(np.pi/180)
    sample_time = 0.01
    Sim_Time_Seconds = 10

    TimeHorizon = 5
    SOLLWERT = 5030.0
    controller = ChessnaMPCController(TimeHorizon,sample_time)
    predictions = []
    time = []
    pid = PID(-1, -0.4, -0.4, setpoint=SOLLWERT)
    pid.output_limits = (-15.0*(np.pi/180), 15.0*(np.pi/180))

    #pid.sample_time = 0.0
    prev_input = 0
    controller.update_SOLL_HIEGHT(SOLLWERT)
    for i in range(round(Sim_Time_Seconds*(1/sample_time))):
        time.append(i*sample_time)
        
        # controls,states = controller.getPredictionControl()
        # y_pred = controller.outputFromPredictionState(states[0])
        # predictions.append(y_pred[1])
        # #print(ctrl*(180/np.pi)
        # response = controller.getSystemResponse(controls[0])

        response = controller.getSystemResponse([input])
        #input = pid(system_output)
        #delta_input = (input-prev_input)
        
        ## angle decreases
        #if delta_input < -60.0*(np.pi/180)*sample_time:
        #    #print(delta_input, " < ",-60.0*(np.pi/180)*sample_time)
        #    input = prev_input - 60.0*(np.pi/180)*sample_time
        ##angle increases
        #if delta_input > 60.0*(np.pi/180)*sample_time:
        #    #print(delta_input, " > ",60.0*(np.pi/180)*sample_time)
        #    input = prev_input + 60.0*(np.pi/180)*sample_time
        #prev_input = input
        #controller.updateSystem()

    outs = np.array(controller.system_outputs)
    output_pitch_angle = outs[:,0]
    output_alt = outs[:,1]
    inputs = controller.system_inputs

    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("alt")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(time,output_alt,label='ALtitude',c="g")
    print(output_alt)
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel(" angle")
    ax2.plot(time,inputs,label="applied inputs",c="c")
    ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    #ax2.set_ylim([-90, 90])

    plt.legend(loc=4)
    plt.show()

    exit()
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

<<<<<<< Updated upstream
import control
def main5():
    system = InvertedPendulimSS()
    input = [1]
    for i in range(50):
        out = system.OneTick(input)
    print(np.shape(system.SystemHistory))
    inputs = system.SystemHistory[:,0]
    out_x = system.SystemHistory[:,1]
    out_angle = system.SystemHistory[:,2]
    fig, ax1 = plt.subplots()
    titlesting = "Simple PID COntrol"
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.plot(out_x,label="system out_x",c="g")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(inputs,label="applied inputs",c="b")
    #ax2.plot(out_angle,label="system out_angle",c="r")
    ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()

#main5()
#main3()
main4()
#main()
#main2()

=======
main()
#main2()
#main3()
>>>>>>> Stashed changes



