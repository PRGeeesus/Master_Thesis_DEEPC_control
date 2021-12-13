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
from SimpleSystems import CessnaSystem, ChessnaMPCController, FederMasseSystem, InvertedPendulimSS, SimpleSystem1,SecondOrderSystem,QuadCopter,Chessna2
import time
from carlaHelper import readFromCSV
import csv
#import control


def main():
    system1 = SimpleSystem1(0,0,1)
    input = 0.5
    for i in range(1,300):
        system1.OneTick(input)
        system1.OneTick((random.random()-0.5)*2)
        #system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
    outputs = system1.SystemHistory[:,1]
    #plt.plot(outputs)
    #plt.show()

    original_data = outputs

    T_ini = 5# über 3 = schwingen, unter 3 = linearer
    T_f = 10
    Controller_settings = {"lambda_s":20,"lambda_g":0.01}
    ctrl = C3.Controller(system1.SystemHistory,T_ini,T_f,1,1,**Controller_settings)
    SOLLWERT = 15
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateIOConstrains([-1],[1],[-np.inf],[np.inf])
   
    ctrl.updateControlCost_R([[1]]) # as I increase, only small changes in control
    ctrl.updateTrackingCost_Q([[1]]) #as I increase, prediction gets closer to SOLLWERT?
    #ctrl.updateReferenceInput([0.5])
    system1.resetSystem()
    predictions_y = []
    applied_inputs = []
    soll = [SOLLWERT]
    for i in range(1,100):
        if i == 50:
            ctrl.updateReferenceWaypoint([10])

        #ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        soll.append(ctrl.y_r[0]) 
        predictions_y.append(y_star[0][0])
        system_output = system1.OneTick(u_star[0][0])
        applied_inputs.append(system1.u)
        ctrl.updateIn_Out_Measures(u_star[0],[system_output])
    
    outputs2 = system1.SystemHistory[:,1]
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])

    fig, ax1 = plt.subplots()
    titlesting = "T_ini:" + str(T_ini) + " T_f:" + str(T_f) +" lg:" + str(ctrl.lambda_g) + " ls:"+ str(ctrl.lambda_s)
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    #ax1.plot(original_data,label='Init Data')
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

def Chessna():
    sample_time = 0.1
    sim_time    = 10
    sys = CessnaSystem(sample_time)
    input = 0.0 * (np.pi/180)
    time = [0]
    for i in range(round(sim_time /sample_time)):
        time.append(i*sample_time)
        sys.OneTick(input)
    
    output_alt = sys.SystemHistory[:,2]
    inputs     = sys.SystemHistory[:,0]

    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("Altitide")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(time,output_alt,label='ALtitude',c="g")
    print(output_alt)
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel(" angle")
    ax2.plot(time,inputs,label="applied inputs",c="c")
    #ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    #ax2.set_ylim([-90, 90])

    plt.legend(loc=4)
    plt.show()

def QuadCopter():
    sys = Chessna2()

    inputs = sys.inputs
    behaviour = sys.system_behaviour
    time = sys.time

    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("Altitide")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(time,behaviour,label='behaviour',c="g")
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel(" angle")
    ax2.plot(time,inputs,label="applied inputs",c="c")
    #ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    #ax2.set_ylim([-90, 90])

    plt.legend(loc=4)
    plt.show()

def Chessna_2():

    timesteps = 0.1
    sim_time = 10
    sys = Chessna2(timesteps,10)
    time = []
    
    for i in range(round(sim_time/timesteps)):

        time.append(i*timesteps)
        ctrl = sys.getControl()
        sys.getSystemresponse(ctrl)
        sys.updateSystem()


    inputs = sys.inputs
    altitude = sys.out_alt
    pitch_angle = sys.out_angle


    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("Altitide")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(time,altitude,label='altitude',c="b")
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel(" angle")
    ax2.step(time,inputs,label="applied inputs",c="g")
    ax2.plot(time,pitch_angle,label="output pitchangle",c="r")
    #ax2.set_ylim([-30, 30])

    plt.legend(loc=4)
    plt.show()

import time
def FederMasse():
    timesteps = 1
    sim_time = 400
    input_force = 5.0
    sys = FederMasseSystem(timesteps)
    timeline = [0.0]
    
    for i in range(round(sim_time/timesteps)):
        if i*timesteps%10 == 0:
            input_force = random.random()
        timeline.append(i*timesteps)
        sys.OneTick(input_force)


    inputs = sys.in_force
    x_out = sys.out_pos
    data = np.hstack([np.reshape(inputs,(len(inputs),1)),np.reshape(x_out,(len(x_out),1))])
    

   
    #### PLOTTING COLLECTED DATA
    # fig, ax1 = plt.subplots()
    # titlesting = "FederMasse System - Sprung"
    # plt.title(titlesting)
    # ax1.set_ylabel("length")
    # #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    # ax1.plot(timeline,x_out,label='x_out',c="g")
    # #ax1.set_ylim([4500, 5500])
    # plt.legend(loc=8)
    # # ...
    # ax2 = ax1.twinx()
    # ax2.set_ylabel(" Force")
    # ax2.plot(timeline,inputs,label="applied input force",c="c")
    # #ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    # #ax2.set_ylim([-90, 90])

    # plt.legend(loc=4)
    # plt.show()

    #### DEEPC PREDICTION PART
    T_ini = 5
    T_f = 20
    settings = {"lambda_s":1,
                "lambda_g":1,
                    "out_constr_lb":[-20],
                    "out_constr_ub":[20],
                    "in_constr_lb":[-np.inf],
                    "in_constr_ub":[np.inf]}

    ctrl = C3.Controller(data,T_ini,T_f,1,1,**settings)
    SOLLWERT = 5
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateControlCost_R([[1]])
    ctrl.updateTrackingCost_Q([[1]])
    #ctrl.updateReferenceInput([0.5])
    sys.resetSystem()

    predictions_y = [0]
    applied_inputs = []
    soll = [SOLLWERT]
    timeline = [0]
    #pid = PID(0.9, 10, 0.1, setpoint=SOLLWERT)
    #pid.output_limits = (-5.0, 5.0)
    #pid.sample_time = 0.005
    
    for i in range(0,100):
        if i == 50:
            SOLLWERT = -3
            ctrl.updateReferenceWaypoint([SOLLWERT])
        #     #pid.setpoint = SOLLWERT
        timeline.append(i)
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        #print(u,u_star)
        #control = pid(sys.out_pos[-1])
        soll.append(SOLLWERT)
        applied_input = u_star[0][0]
        system_output = sys.OneTick(applied_input)
        #system_output = sys.OneTick(control)
        predictions_y.append(y_star[0][0])
        applied_inputs.append(applied_input)
        #applied_inputs.append(control)
        ctrl.updateIn_Out_Measures([applied_input],[system_output])
        ctrl.updateReferenceInput([applied_input])
    
    inputs = sys.in_force
    x_out = sys.out_pos
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])

    fig, ax1 = plt.subplots()
    titlesting = "DEEPC CONTROLLED to " + str(SOLLWERT) + ""
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='SOLLWERT',c="y")
    ax1.plot(timeline,predictions_y,label='predictions',c="r")
    ax1.plot(timeline,x_out,label="system behaviour",c="g")
    
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(timeline,inputs,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])

    plt.legend(loc=4)
    plt.show()

def ControlWithPID():
    pid = PID(0.3, 0.1, 0.00, setpoint=SOLLWERT)
    pid.output_limits = (-1.0, 1.0)
    pid.sample_time = 0.00
    control = pid(system_output)

#main5()
#main3()
#main4()
#main()
#main2()
#Chessna()
Chessna_2()
#FederMasse()




