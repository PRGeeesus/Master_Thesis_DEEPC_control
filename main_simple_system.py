# goal is to use DEEPC to predict a simple integrator:
# x(k+1) = x(k) + I*u(k)
# step1: y(0) = x(0)
#        x(1) = y(0) + T * u(0)
#setp2: y(1) = x(1) + T * u(1)
#       x(2) = y(1)
#setpk  y(k) = x(k) + T* u(k)
#       x(k +1) = y(k)

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import matplotlib.ticker as mtick
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib

import random
import DeePC_OSQP as DeePC
import numpy as np
import SimpleSystems
import time
from carlaHelper import readFromCSV
import csv
import time

from simple_pid import PID
import SimpleSystemsAnimations
import SimpleSystems
from SimpleSystems import InvertedPendulum_MPC,ISystem, FederMasseSystem, Chessna2, FederMasseSystem_MPC,InvertedPendulumSS ,saveAsCSV ,readFromCSV
from Voltage_Controller import PC_INTERFACE

import BLDC_MOTOR

def ISystem_1(l_s,l_g,Q,R,scout = False):
    system1 = ISystem(0,0,1)
    input = 0.0
    time_t = [0]
    
    for i in range(1,150):
        
        #system1.OneTick(input)
        #if i > 15 < 30:
        #    input = 0.0
        #if i >=30:
        #    input = 1
        
        if i%50 ==0:
            input = (random.random()-0.5)*2
        system1.OneTick(input)
        #system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
        time_t.append(i)
    
    filename = "I-SYSTEM-DATA"
    #saveAsCSV(filename,system1.SystemHistory)
    
    #data_2 = readFromCSV(filename)
    data_2 = system1.SystemHistory
    print("Data Length: ",len(data_2))

    outputs = system1.SystemHistory[:,1]
    inputs = system1.SystemHistory[:,0]

    """
    fig, ax1 = plt.subplots()
    plt.title("Integrating System")
    ax1.set_xlabel('time')
    ax1.set_ylabel('Output', c='r')
    ax1.plot(time_t, outputs, c='r',label = "Output",linewidth=3)
    ax1.tick_params(axis='y', labelcolor='c')
    ax1.legend(loc=2)
    plt.legend()
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:blue'
    ax2.set_ylabel('Input', color=color)  # we already handled the x-label with ax1
    ax2.plot(time_t, inputs, color=color, label = "Input")
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    ax2.legend(loc=1)
    plt.show()
    """

    original_data = outputs

    T_ini =3
    T_f = 10
    Controller_settings = {"lambda_s":l_s,
                           "lambda_g":l_g,
                            "out_constr_lb":[-1000],
                            "out_constr_ub":[1000],
                            "in_constr_lb":[-1],
                            "in_constr_ub":[1],
                            "regularize":True,
                            "verbose":False}

    ctrl = DeePC.Controller(data_2,T_ini,T_f,1,1,**Controller_settings)
    SOLLWERT = 5
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
 
    ctrl.updateControlCost_R([[R]]) # as I increase, only small changes in control
    ctrl.updateTrackingCost_Q([[Q]]) #as I increase, prediction gets closer to SOLLWERT?
    
    
    system1.resetSystem()

    predictions_y = [0]
    applied_inputs = []
    soll = [SOLLWERT]
    prediction = 0
    control_input = 0
    for i in range(1,100):
        if i == 50:
            ctrl.updateReferenceWaypoint([15])

        #ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
        if i > T_ini+2:
            u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
            prediction = y_star[0][0]
            control_input = u_star[0][0]
        soll.append(ctrl.y_r[0]) 
        predictions_y.append(prediction)
        system_output = system1.OneTick(control_input)
        applied_inputs.append(system1.u)
        ctrl.updateIn_Out_Measures([control_input],[system_output])
        #ctrl.test_g_validity()
    
    outputs2 = system1.SystemHistory[:,1]
    track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(outputs2[20:],predictions_y[20:])
    control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll[20:],outputs2[20:])
    if scout: return track_mean,control_mean
    
    fig, ax1 = plt.subplots()
    titlesting = "I System DEEPC Controlled to" + str(SOLLWERT)
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.set_xlabel("time")
    #ax1.plot(original_data,label='Init Data')
    ax1.plot(soll,label='setpoint',c="y")
    ax1.plot(predictions_y,label='prediction',c="r")
    ax1.plot(outputs2,label="system behaviour",c="g")
    
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(applied_inputs,label="applied inputs",c="b")
    ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()
    
def ISystem_scout_lg_ls():
    tracking_means = []
    control_means = []
    
    #ls_range = range(1,ls_scale,ls_factor)
    ls_range  = [10**i for i in range(1,7)]

    lg_range = [i for i in range(1,10,1)]
    
    for i in ls_range:
        temp1 = []
        temp2 = []
        for j in lg_range:
            tm,cm = ISystem_1(i,j,1,1,True)
            temp1.append(tm)
            temp2.append(cm)
        tracking_means.append(temp1)
        control_means.append(temp2)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    tracking_means = np.asarray(tracking_means)
    control_means = np.asarray(control_means)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    
    filename = "tracking_mean_ls_lg_scan_i_system"
    saveAsCSV(filename,tracking_means)
    filename = "control_means_ls_lg_scan_i_system"
    saveAsCSV(filename,control_means)

    """
    fig = plt.figure(figsize=(14,6))

    x, y = np.meshgrid(lg_range,ls_range)
    print(np.shape(x),np.shape(y),np.shape(tracking_means))
    # surface_plot with color grading and color bar
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    p = ax.plot_surface(x,y, tracking_means, rstride=1, cstride=1, cmap=matplotlib.cm.coolwarm, linewidth=0, antialiased=False)
    cb = fig.colorbar(p, shrink=0.5)
    plt.show()

    """
    # MEAN TRACKING DIFFERENCE

    fig = plt.figure()
    ax = fig.add_subplot(111)
    #norm = matplotlib.colors.LogNorm(tracking_means.mean() + 0.5 * tracking_means.std(), tracking_means.max(), clip='True')
    #im = ax.imshow(tracking_means, cmap='YlGn', norm=norm, origin="lower")
    im = ax.imshow(tracking_means,cmap='YlGn')
    ax.set_ylabel("lambda s",fontsize = 16)
    ax.set_xlabel("lambda g",fontsize = 16)
    ax.set_yticks(np.arange(tracking_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(tracking_means.shape[1]), minor=False)



    plt.colorbar(im)

    ax.set_yticklabels(['{:.1E}'.format(y) for y in ls_range]); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in lg_range],rotation=45); # use LaTeX formatted labels

    

    ax.invert_yaxis()
    ax.set_title("Prediction: Lambda g vs Lambda s",fontsize = 16)
    fig.tight_layout()
    plt.show()
    

    # MEAN COntrol Difference

    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(control_means,cmap='YlGn')
    ax.set_ylabel("lambda s",fontsize = 16)
    ax.set_xlabel("lambda g",fontsize = 16)
    ax.set_yticks(np.arange(control_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(control_means.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in ls_range]); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in lg_range],rotation=45); # use LaTeX formatted labels


    ax.invert_yaxis()
    ax.set_title("Setpoint: Lambda g vs Lambda s",fontsize = 16)
    fig.tight_layout()
    plt.show()

def ISystem_scout_Q_R():
    tracking_means = []
    control_means = []
    ls_factor = 20
    ls_scale = 200
    
    #ls_range = range(1,ls_scale,ls_factor)
    Q  = [10**i for i in range(1,8)]
    R  = [i for i in range(1,100,10)]
    
    for i in R:
        temp1 = []
        temp2 = []
        for j in Q:
            tm,cm = ISystem_1(1,1,j,i,True)
            temp1.append(tm)
            temp2.append(cm)
        tracking_means.append(temp1)
        control_means.append(temp2)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    tracking_means = np.asarray(tracking_means)
    control_means = np.asarray(control_means)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    
    filename = "tracking_mean_ls_lg_scan_i_system"
    saveAsCSV(filename,tracking_means)
    filename = "control_means_ls_lg_scan_i_system"
    saveAsCSV(filename,control_means)


    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(tracking_means,cmap='YlGn')
    ax.set_ylabel("R",fontsize = 16,rotation=0,weight = 'bold',labelpad=25)
    ax.set_xlabel("Q",fontsize = 16,weight = 'bold')
    ax.set_yticks(np.arange(tracking_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(tracking_means.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in R],fontsize = 14); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in Q],rotation=45,fontsize = 14); # use LaTeX formatted labels


    ax.invert_yaxis()
    ax.set_title("Prediction: Q vs R",fontsize = 20,weight = 'bold',pad = 25)
    fig.tight_layout()
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(control_means,cmap='YlGn')
    ax.set_ylabel("R",fontsize = 16,rotation=0,weight = 'bold',labelpad=25)
    ax.set_xlabel("Q",fontsize = 16,weight = 'bold')
    ax.set_yticks(np.arange(control_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(control_means.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in R],fontsize = 14); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in Q],rotation=45,fontsize = 14); # use LaTeX formatted labels


    ax.invert_yaxis()
    ax.set_title("Set-point: Q vs R",fontsize = 20,weight = 'bold',pad = 25)
    fig.tight_layout()
    plt.show()

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
    ctrl = DEEPC.Controller(system1.SystemHistory,T_ini,T_f,1,1)
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

def InvertedPendulum_MPC_WITHOUT_CART():
    timesteps = 0.1
    sim_time = 20
    time_horizon = int(2/timesteps)
    SOLLWERT = 1
    input_force = 5.0
    sys = InvertedPendulum_MPC(timesteps,time_horizon,SOLLWERT)
    
    
    # SimpleSystems.saveAsCSV("Feder_Masse_"+str(len(data2)), data2)
    #data = SimpleSystems.readFromCSV("Feder_Masse_101")
   
    ### PLOTTING COLLECTED DATA
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
    timeline = []
    applied_inputs = []
    system_outputs = []
    soll = []
    control = sys.getControl()
    for i in range(0,int(sim_time/timesteps)): 
        timeline.append(i)
        
        out = sys.getSystemresponse(control)
        control = sys.getControl()
        sys.updateSystem()

        soll.append(SOLLWERT)
        applied_input = control
        system_output = out
        applied_inputs.append(applied_input)
        system_outputs.append(system_output)
        #ctrl.test_g_validity()

    #print(applied_inputs)
    #print(system_outputs)
    outputs_x = np.asarray(system_outputs)[:,0]
    outputs_angle = np.asarray(system_outputs)[:,1]
    fig, ax1 = plt.subplots()
    titlesting = "MPC CONTROLLED to " + str(SOLLWERT) + ""
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='SOLLWERT',c="y")
    ax1.plot(timeline,outputs_x,label='out x',c="r")
    ax1.plot(timeline,outputs_angle,label='out angle',c="g")
   
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(timeline,applied_inputs,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])

    plt.legend(loc=4)
    plt.show()

def InvertedPendulum_OnCart():
    timestep = 0.1
    simtime = 10
    sys = SimpleSystems.InvertedPendulum_WithCart(timestep,10)
    control = [0.2]
    time = [0]
    sys.getSystemresponse(control)
    sys.updateSystem()
    for i in range(int(simtime/timestep)):
        control = sys.getControl()
        sys.getSystemresponse(control)
        sys.updateSystem()
        time.append(i*timestep)
        if sys.x0[0] > 10:
            break
    out_x = sys.out_x
    out_angle = sys.out_angle
    ins = sys.inputs

    fig, ax1 = plt.subplots()
    titlesting = "Inverted Pendulum"
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    ax1.plot(time,out_x,label="system out x",c="g")
    ax1.plot(time,out_angle,label="system out angle",c="r")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(time,ins,label="Input Force",c="b")
    #ax2.plot(out_angle,label="system out_angle",c="r")
    #ax2.set_ylim([-2, 2])

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

def QuadCopter_system():
    sys = QuadCopter(0.1)

    inputs = sys.inputs
    behaviour = sys.system_behaviour
    time = sys.time

    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("Altitide")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(time,behaviour,label='z-height',c="g")
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
    SOLLWERT = 5020
    sys = Chessna2(timesteps,10)
    time = []
    soll = []
    for i in range(round(sim_time/timesteps)):
        soll.append(SOLLWERT)
        time.append(i*timesteps)
        ctrl = sys.getControl()
        sys.getSystemresponse(ctrl)
        sys.updateSystem()


    inputs = np.asarray(sys.inputs)
    inputs = np.reshape(sys.inputs,(len(sys.inputs),1))
    altitude = np.reshape(sys.out_alt,(len(sys.out_alt),1))

    pitch_angle = sys.out_angle

    data = np.hstack([inputs,altitude])
    #print("Recorded data:",inputs,altitude)
    #track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(altitude,predictions_y)
    control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,altitude)

    fig, ax1 = plt.subplots()
    titlesting = "Title"
    plt.title(titlesting)
    ax1.set_ylabel("Altitide")
    ax1.plot(time,soll,label='Sollwert.',c="y")
    ax1.plot(time,altitude,label='Altitude',c="g")
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("angle")
    ax2.step(time,inputs*sys.factor_rad_to_deg,label="applied inputs",c="b")
    #ax2.plot(time,pitch_angle,label="output pitchangle",c="r")
    #ax2.set_ylim([-30, 30])

    plt.legend(loc=4)
    plt.show()

    ### DEEPC PART:
    T_ini = 3
    T_f = 10
    settings = {"lambda_s":1,
                "lambda_g":1,
                "out_constr_lb":[0],
                "out_constr_ub":[6000],
                "in_constr_lb":[-15*sys.factor_deg_to_rad],
                "in_constr_ub":[15*sys.factor_deg_to_rad]}

    ctrl = DEEPC.Controller(data,T_ini,T_f,1,1,**settings)
    SOLLWERT = 5020.0
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateControlCost_R([[1]])
    ctrl.updateTrackingCost_Q([[1]])
    #ctrl.updateReferenceInput([0.5])
    sys.resetSystem()

    predictions_y = []
    soll = []
    timeline = []
    #pid = PID(0.9, 10, 0.1, setpoint=SOLLWERT)
    #pid.output_limits = (-5.0, 5.0)
    #pid.sample_time = 0.005
    Control_input = 1.0*sys.factor_deg_to_rad
    prediction = 0
    
    for i in range(0,100):
        timeline.append(i)
        if i > T_ini +3:
            u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
            Control_input = u[0][0]
            prediction = y_star[0][0]
        #print(u,u_star)
        #control = pid(sys.out_pos[-1])
        soll.append(SOLLWERT)
        applied_input = -Control_input
        system_output = sys.getSystemresponse([applied_input])
        predictions_y.append(prediction)
        ctrl.updateIn_Out_Measures([applied_input],[system_output[0]])
        #ctrl.updateReferenceInput([applied_input])
        #ctrl.test_g_validity()
    
    inputs = np.asarray(sys.inputs)
    inputs = np.reshape(sys.inputs,(len(sys.inputs),1))
    altitude = np.reshape(sys.out_alt,(len(sys.out_alt),1))
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])
    track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(altitude,predictions_y)
    control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,altitude)


    fig, ax1 = plt.subplots()
    titlesting = "DEEPC CONTROLLED to " + str(SOLLWERT) + ""
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='SOLLWERT',c="y")
    ax1.plot(timeline,predictions_y,label='Predictions',c="r")
    ax1.plot(timeline,altitude,label="Altitude",c="g")
    
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(timeline,inputs*sys.factor_rad_to_deg,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])

    plt.legend(loc=4)
    plt.show()

def FederMasse(l_s,l_g,Q,R,scout = False):
    timesteps = 1
    sim_time = 200
    input_force = 5.0
    sys = FederMasseSystem(timesteps)
    timeline = []
    
    for i in range(round(sim_time/timesteps)):
        if i == 30:
            input_force = random.random()*5
        if i == 60:
            input_force = random.random()*5
        timeline.append(i*timesteps)
        sys.OneTick(input_force)


    inputs = sys.in_force
    x_out = sys.out_pos
    data = np.hstack([np.reshape(inputs,(len(inputs),1)),np.reshape(x_out,(len(x_out),1))])
    
    # SimpleSystems.saveAsCSV("Feder_Masse_"+str(len(data2)), data2)
    #data = SimpleSystems.readFromCSV("Feder_Masse_101")
    #data = data[:300]
    """
    ### PLOTTING COLLECTED DATA
    fig, ax1 = plt.subplots()
    titlesting = "FederMasse System - Sprung"
    plt.title(titlesting)
    ax1.set_ylabel("Position x")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(timeline,x_out,label='Output Position',c="g")
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc='upper center')
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Force")
    ax2.plot(timeline,inputs,label="Input Force",c="c")
    #ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    #ax2.set_ylim([-90, 90])

    plt.legend(loc='upper right')
    plt.show()
    """
    #### DEEPC PREDICTION PART
    print("len data:", len(data))
    T_ini = 4
    T_f = 15
    settings = {"lambda_s":l_s,
                "lambda_g":l_g,
                "out_constr_lb":[-np.inf],
                "out_constr_ub":[np.inf],
                "in_constr_lb":[-30],
                "in_constr_ub":[30],
                "regularize":True,
                "verbose":False}

    ctrl = DeePC.Controller(data,T_ini,T_f,1,1,**settings)
    SOLLWERT = 5
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateTrackingCost_Q([[Q]])
    ctrl.updateControlCost_R([[R]])

    sys.resetSystem()

    predictions_y = [0]
    prediction_1 = []
    prediction_2 = []
    prediction_3 = []
    one_time_offset = 10
    x_range_prediction_1 = [i for i in range(T_f+one_time_offset*1,T_f+T_f+one_time_offset*1)]
    x_range_prediction_2 = [i for i in range(T_f+one_time_offset*2,T_f+T_f+one_time_offset*2)]
    x_range_prediction_3 = [i for i in range(T_f+one_time_offset*3,T_f+T_f+one_time_offset*3)]
    applied_inputs = []
    soll = [SOLLWERT]
    timeline = [0]
    #pid = PID(0.9, 10, 0.1, setpoint=SOLLWERT)
    #pid.output_limits = (-5.0, 5.0)
    #pid.sample_time = 0.005
    Control_input = 0
    prediction = 0
    u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
    for i in range(0,200): 
        if i == 100:
            SOLLWERT = -5
            ctrl.updateReferenceWaypoint([SOLLWERT])
        #     #pid.setpoint = SOLLWERT
        timeline.append(i)
        if i > T_f +3:
            u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
            Control_input = u[0][0]
            prediction = y_star[0][0]
        if i == T_f + one_time_offset*1:
            prediction_1 = y_star[:,0]
        if i == T_f + one_time_offset*2:
            prediction_2 = y_star[:,0]
        if i == T_f + one_time_offset*3:
            prediction_3 = y_star[:,0]
        


        #print(u,u_star)
        #control = pid(sys.out_pos[-1])
        soll.append(SOLLWERT)
        applied_input = Control_input
        system_output = sys.OneTick(applied_input)
        predictions_y.append(prediction)
        applied_inputs.append(applied_input)
        ctrl.updateIn_Out_Measures([applied_input],[system_output])
        ctrl.updateReferenceInput([applied_input])
        #ctrl.test_g_validity()
    
    inputs = sys.in_force
    x_out = sys.out_pos
    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])
    track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(x_out,predictions_y)
    control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,x_out)
    if scout: return track_mean,control_mean

    fig, ax1 = plt.subplots()
    titlesting = "DeePC control"
    plt.title(titlesting,fontsize = 16)
    ax1.set_ylabel("Outputs",fontsize = 16)
    ax1.set_xlabel("Time",fontsize = 16)
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='set point',c="y")
    #ax1.plot(timeline,predictions_y,label='predictions',c="r")
    ax1.plot(timeline,x_out,label="system behaviour",c="g")
    #ax1.plot(x_range_prediction_1,prediction_1,label='snapshot prediction 1',c="purple")
    #ax1.plot(x_range_prediction_2,prediction_2,label='snapshot prediction 2',c="purple")
    #ax1.plot(x_range_prediction_3,prediction_3,label='snapshot prediction 3',c="purple")
    
    #plt.legend(loc='lower right')
    plt.legend(loc='upper right')
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs",fontsize = 16)
    ax2.plot(timeline,inputs,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])

    plt.legend(loc='right')
    plt.show()


def SMDSystem_scout_lg_ls():
    tracking_means = []
    control_means = []
    
    #ls_range = range(1,ls_scale,ls_factor)
    ls_range  = [10**i for i in range(1,10)]

    lg_range = [10**i for i in range(1,10)]
    
    for i in ls_range:
        temp1 = []
        temp2 = []
        for j in lg_range:
            tm,cm = FederMasse(i,j,1,1,True)
            temp1.append(tm)
            temp2.append(cm)
        tracking_means.append(temp1)
        control_means.append(temp2)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    tracking_means = np.asarray(tracking_means)
    control_means = np.asarray(control_means)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    
    filename = "tracking_mean_ls_lg_scan_SMD_system"
    saveAsCSV(filename,tracking_means)
    filename = "control_means_ls_lg_scan_SMD_system"
    saveAsCSV(filename,control_means)

    """
    fig = plt.figure(figsize=(14,6))

    x, y = np.meshgrid(lg_range,ls_range)
    print(np.shape(x),np.shape(y),np.shape(tracking_means))
    # surface_plot with color grading and color bar
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    p = ax.plot_surface(x,y, tracking_means, rstride=1, cstride=1, cmap=matplotlib.cm.coolwarm, linewidth=0, antialiased=False)
    cb = fig.colorbar(p, shrink=0.5)
    plt.show()

    """
    # MEAN TRACKING DIFFERENCE

    fig = plt.figure()
    ax = fig.add_subplot(111)
    #norm = matplotlib.colors.LogNorm(tracking_means.mean() + 0.5 * tracking_means.std(), tracking_means.max(), clip='True')
    #im = ax.imshow(tracking_means, cmap='YlGn', norm=norm, origin="lower")

    # 
    data = tracking_means[:][:]
    im = ax.imshow(data,cmap='YlGn')
    ax.set_ylabel("lambda s",fontsize = 16)
    ax.set_xlabel("lambda g",fontsize = 16)
    ax.set_yticks(np.arange(data.shape[0]), minor=False)
    ax.set_xticks(np.arange(data.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in ls_range]); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in lg_range],rotation=45); # use LaTeX formatted labels

    ax.invert_yaxis()
    ax.set_title("Prediction: Lambda g vs Lambda s",fontsize = 16)
    fig.tight_layout()
    plt.show()
    

    # MEAN COntrol Difference

    data = control_means
    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(data,cmap='YlGn')
    ax.set_ylabel("lambda s",fontsize = 16)
    ax.set_xlabel("lambda g",fontsize = 16)
    ax.set_yticks(np.arange(data.shape[0]), minor=False)
    ax.set_xticks(np.arange(data.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in ls_range]); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in lg_range],rotation=45); # use LaTeX formatted labels 

    ax.invert_yaxis()
    ax.set_title("Set-point: Lambda g vs Lambda s",fontsize = 16)
    fig.tight_layout()
    plt.show()

def SMDSystem_scout_Q_R():
    tracking_means = []
    control_means = []
    ls_factor = 20
    ls_scale = 200
    
    #ls_range = range(1,ls_scale,ls_factor)
    Q  = [i for i in range(1,1000,100)]
    R  = [i for i in range(1,1000,100)]
    print(Q,R)
    
    for i in R:
        temp1 = []
        temp2 = []
        for j in Q:
            tm,cm = FederMasse(1,1,j,i,True)
            temp1.append(tm)
            temp2.append(cm)
        tracking_means.append(temp1)
        control_means.append(temp2)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    tracking_means = np.asarray(tracking_means)
    control_means = np.asarray(control_means)
    print("tracking_means len:",len(tracking_means)," type: ",type(tracking_means), " shape: ",np.shape(tracking_means))
    
    filename = "tracking_mean_ls_lg_scan_i_system"
    saveAsCSV(filename,tracking_means)
    filename = "control_means_ls_lg_scan_i_system"
    saveAsCSV(filename,control_means)


    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(tracking_means,cmap='YlGn')
    ax.set_ylabel("R",fontsize = 16,rotation=0,weight = 'bold',labelpad=25)
    ax.set_xlabel("Q",fontsize = 16,weight = 'bold')
    ax.set_yticks(np.arange(tracking_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(tracking_means.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in R],fontsize = 14); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in Q],rotation=45,fontsize = 14); # use LaTeX formatted labels


    ax.invert_yaxis()
    ax.set_title("Prediction: Q vs R",fontsize = 20,weight = 'bold',pad = 25)
    fig.tight_layout()
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(control_means,cmap='YlGn')
    ax.set_ylabel("R",fontsize = 16,rotation=0,weight = 'bold',labelpad=25)
    ax.set_xlabel("Q",fontsize = 16,weight = 'bold')
    ax.set_yticks(np.arange(control_means.shape[0]), minor=False)
    ax.set_xticks(np.arange(control_means.shape[1]), minor=False)
    plt.colorbar(im)
    ax.set_yticklabels(['{:.1E}'.format(y) for y in R],fontsize = 14); # use LaTeX formatted labels
    ax.set_xticklabels(['{:.1E}'.format(x) for x in Q],rotation=45,fontsize = 14); # use LaTeX formatted labels


    ax.invert_yaxis()
    ax.set_title("Set-point: Q vs R",fontsize = 20,weight = 'bold',pad = 25)
    fig.tight_layout()
    plt.show()



def FederMasse_MPC():

    timestep = 0.1
    simtime = 15
    TIME_HORIZON = int(4/timestep) #seconds
    SOLLWERT = 5
    sys = SimpleSystems.FederMasseSystem_MPC(timestep,TIME_HORIZON,SOLLWERT)
    control = [0.0]
    time = [0]
    soll = [SOLLWERT]
    sys.getSystemresponse(control)
    sys.updateSystem()
    for i in range(int(simtime/timestep)):
        soll.append(SOLLWERT)
        control = sys.getControl()
        sys.getSystemresponse(control)
        sys.updateSystem()
        time.append(i*timestep)


    out_x = sys.out_x
    ins = sys.inputs
    predicitons = np.asarray(sys.prediction)
    
    f_predicitons = np.asarray(sys.first_prediction)
    #predicitons = np.insert(predicitons,0,[None for i in range(TIME_HORIZON+1)])
    #predicitons = predicitons[:len(predicitons)-TIME_HORIZON]


    fig, ax1 = plt.subplots()
    titlesting = "MPC Controlled Spring-Mass-Damper System"
    plt.title(titlesting)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position x [m]")
    ax1.plot(time,out_x,label="System position",c="g")
    #ax1.plot(time,predicitons,label="Model prediction",c="r")
    ax1.plot(time,soll,label="Reference [m]",c="tab:orange")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Force F[N]")
    ax2.plot(time,ins,label="Input Force",c="b")
    #ax2.plot(out_angle,label="system out_angle",c="r")
    #ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()

def FederMasse_PID():

    timestep = 0.01
    simtime = 15
    TIME_HORIZON = int(4/timestep) #seconds
    SOLLWERT = 5.0
    sys = SimpleSystems.FederMasseSystem(timestep)
    control = [0.0]
    time = []
    time.append(0)
    soll = [SOLLWERT]
    pid = PID(1, 0.6, 0.05, setpoint=SOLLWERT)
    pid.output_limits = (-20, 20)
    #pid.sample_time = timestep

    sys_out = sys.OneTick(0)

    for i in range(int(simtime/timestep)):
        soll.append(SOLLWERT)
        
        control = pid(sys_out)
        #print("control",control,control[0])
        sys_out = sys.OneTick(control[0])
        time.append(i*timestep)


    out_x = sys.out_pos
    ins = sys.in_force
    
    #predicitons = np.insert(predicitons,0,[None for i in range(TIME_HORIZON+1)])
    #predicitons = predicitons[:len(predicitons)-TIME_HORIZON]


    fig, ax1 = plt.subplots()
    titlesting = "MPC Controlled Spring-Mass-Damper System"
    plt.title(titlesting)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position x [m]")
    ax1.plot(time,out_x,label="System position",c="g")
    ax1.plot(time,soll,label="Reference [m]",c="tab:orange")
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Force F[N]")
    ax2.plot(time,ins,label="Input Force",c="b")
    #ax2.plot(out_angle,label="system out_angle",c="r")
    #ax2.set_ylim([-2, 2])

    plt.legend(loc=4)
    plt.show()

def ControlWithPID():
    pid = PID(0.3, 0.1, 0.00, setpoint=SOLLWERT)
    pid.output_limits = (-1.0, 1.0)
    pid.sample_time = 0.00
    control = pid(system_output)

def PendulumWithCart_Animated():
    cart_animation = SimpleSystemsAnimations.PendulumAnimation()
    x = 0
    while True:

        cart_animation.updateAnimation(x,1)
        x += 1

def BLDC_MOTOR_CONTROL():
    M = BLDC_MOTOR.PC_INTERFACE()
    M.verbose = True
    M.SetMotorStill()
    time.sleep(1)
    M.SetMotorInitRotation()
    time.sleep(1)
 
    other_range = [i for i in range(1310,1600,1)]
    data = []
    for i in range(1000):
        if i%250 == 0:
            set_val = random.randint(other_range[0], other_range[-1])
            M.setRPM(set_val)
        value = M.ESC_RW()
        data.append([set_val,value])

    data = np.asarray(data)
   
    filename = "MOTOR_RAMP_UP"
    saveAsCSV(filename,data)
    data_2 = readFromCSV(filename)
    data_2 = data_2[:100]
    print("DATA LENGTH:",len(data_2))
    M.SetMotorStill()
    time.sleep(1)
    M.SetMotorInitRotation()
    time.sleep(8)

    T_ini = 2
    T_f = 5
    settings = {"lambda_s":1,
                "lambda_g":1,
                "out_constr_lb":[0],
                "out_constr_ub":[400],
                "in_constr_lb":[1250*2],
                "in_constr_ub":[1600*2],
                "verbose":False}

    ctrl = DeePC.Controller(data,T_ini,T_f,1,1,**settings)

    SOLLWERT = 70
    ctrl.updateReferenceWaypoint([SOLLWERT])
    #ctrl.updateReferenceInput([0.5])
    ctrl.updateControlCost_R([[1]])
    ctrl.updateTrackingCost_Q([[1]])
    #ctrl.updateReferenceInput([0.5])
    
    predictions_y = [0]
    applied_inputs = [0]
    system_outputs = [0]
    soll = [SOLLWERT]
    timeline = [0]
    #pid = PID(0.9, 10, 0.1, setpoint=SOLLWERT)
    #pid.output_limits = (-5.0, 5.0)
    #pid.sample_time = 0.005
    Control_input = M.cc_range[10]
    prediction = 0
    #u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
    for i in range(0,12): 
        timeline.append(i)
        if i > T_f :
            u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
            Control_input = u[0][0]
            prediction = y_star[0][0]
        else:
            Control_input += 1
        #print(u,u_star)
        soll.append(SOLLWERT)
        applied_input = Control_input
        M.setRPM(applied_input)
        system_out = M.ESC_RW()
        print(i," OUT: ",system_out," IN: ",applied_input)
        system_outputs.append(system_out)

        predictions_y.append(prediction)
        applied_inputs.append(applied_input)
        ctrl.updateIn_Out_Measures([applied_input],[system_out])
        ctrl.updateReferenceInput([applied_input])
        #ctrl.test_g_validity()
    

    #print("u and y: ", [applied_inputs[i] for i in range(10)],
    #                    [outputs2[i] for i in range(10)])
    #track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(x_out,predictions_y)
    #control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,x_out)


    fig, ax1 = plt.subplots()
    titlesting = "DEEPC CONTROLLED to " + str(SOLLWERT) + ""
    plt.title(titlesting)
    ax1.set_ylabel("Outputs")
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='SOLLWERT',c="y")
    ax1.plot(timeline,predictions_y,label='predictions',c="r")
    ax1.plot(timeline,system_outputs,label="system behaviour",c="g")
    
    plt.legend(loc=8)
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs")
    ax2.plot(timeline,applied_inputs,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])

    plt.legend(loc=4)
    plt.show()

def AirFlow():
    box = PC_INTERFACE()
    value = box.SetVoltage(1)
    time.sleep(0.5)
    value = box.SetVoltage(1023)
    time.sleep(0.5)
    value = box.SetVoltage(0)
    time.sleep(2)
    
    timesteps = 1
    sim_time = 300
    input_Voltage = 5.0

    recording_timeline = []
    recording_inputs = []
    recording_outputs = []
    """
    for i in range(int(round(sim_time/timesteps))):
        
        if i > 200:
            if i%30 == 0:
                input_Voltage = random.random()*1023
        if i <= 200:
            input_Voltage = i

        recording_inputs.append(input_Voltage)
        recording_timeline.append(i*timesteps)
        value = box.SetVoltage(input_Voltage)
        recording_outputs.append(value)


    data = np.hstack([np.reshape(recording_inputs,(len(recording_inputs),1)),
                      np.reshape(recording_outputs,(len(recording_outputs),1))])
    """
    data_name = "Voltage_to_pressure_200"
    #SimpleSystems.saveAsCSV(data_name, data)
    data = SimpleSystems.readFromCSV(data_name)

    """
    ### PLOTTING COLLECTED DATA
    fig, ax1 = plt.subplots()
    titlesting = "Voltage"
    plt.title(titlesting)
    ax1.set_ylabel("Pressure")
    #ax1.plot(time,predictions,label='ALt_Pred.',c="r")
    ax1.plot(data[:,1],label='Pressure Output',c="g")
    #ax1.set_ylim([4500, 5500])
    plt.legend(loc='upper center')
    # ...
    ax2 = ax1.twinx()
    ax2.set_ylabel("Voltage")
    ax2.plot(data[:,0],label="Voltage",c="c")
    #ax2.plot(time,output_pitch_angle,label="output pitchangle",c="b")
    #ax2.set_ylim([-90, 90])

    plt.legend(loc='upper right')
    plt.show()
    """
    
    
    #### DEEPC PREDICTION PART
    #print("len data:", len(data)," data: ",data,"shape: ",np.shape(data))
    # lambda_s":100000,
    # lambda_g":1000000
    T_ini   =  2
    T_f     = 15
    settings = {"lambda_s":100000,
                "lambda_g":1000000,
                "out_constr_lb":[0],
                "out_constr_ub":[1000],
                "in_constr_lb":[1],
                "in_constr_ub":[1023],
                "regularize":True,
                "verbose":False}


    # Set up the Controller
    ctrl = DeePC.Controller(data,T_ini,T_f,1,1,**settings)
    SOLLWERT = 400
    ctrl.updateReferenceWaypoint([SOLLWERT])
    ctrl.updateTrackingCost_Q([[300]])
    ctrl.updateControlCost_R([[1]])

    predictions_y = [0]
    applied_inputs = [0]
    system_outputs = [0]
    soll = [SOLLWERT]
    timeline = [0]
    ##for PID CONTROL
    #pid = PID(1, 1, 0.1, setpoint=SOLLWERT)
    #pid.output_limits = (0, 1024)
    #pid.sample_time = 0.1
    #system_output = 0

    Control_input = 0
    prediction = 0
    u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
    for i in range(0,200):
        timeline.append(i)
        if i > T_f +3:
            u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
            Control_input = u[0][0]
            prediction = y_star[0][0]
        

        #Control_input = pid(system_output)
        

        applied_input = Control_input
        system_output = box.SetVoltage(applied_input)
        # record:
        soll.append(SOLLWERT)
        predictions_y.append(prediction)
        applied_inputs.append(applied_input)
        system_outputs.append(system_output)
        # update controller
        ctrl.updateIn_Out_Measures([applied_input],[system_output])

        # user output:
        print(i,": in: ",applied_input, " out: ", system_output, " prediction: ",prediction)
        time.sleep(0.01)
    

    #track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(system_outputs,predictions_y)
    #control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,system_outputs)
    #if scout: return track_mean,control_mean

    fig, ax1 = plt.subplots()
    titlesting = "DeePC control"
    plt.title(titlesting,fontsize = 16)
    ax1.set_ylabel("Outputs[V]",fontsize = 16)
    ax1.set_xlabel("Time",fontsize = 16)
    #ax1.plot(data[:,1],label='Init Data')
    ax1.plot(timeline,soll,label='set point',c="y")
    ax1.plot(timeline,predictions_y,label='predictions',c="r")
    ax1.plot(timeline,system_outputs,label="system behaviour",c="g")
    plt.legend(loc='upper right')

    # PLOT ON RIGHT AXIS
    ax2 = ax1.twinx()
    ax2.set_ylabel("Inputs[V]",fontsize = 16)
    ax2.plot(timeline,applied_inputs,label="applied inputs",c="b")
    #ax2.plot(data[:,0],label='Init inputs')
    #ax2.set_ylim([-7, 7])
    plt.legend(loc='right')

    plt.show()

def scientific(x, pos):
    # x:  tick value - ie. what you currently see in yticks
    # pos: a position - ie. the index of the tick (from 0 to 9 in this example)
    return '%.1E' % x

def BOX_SHOW_OUTPUT_CURVE():
    pwm_values = []
    speed_values = []
    voltage_values = []
    box = PC_INTERFACE()
    value = box.SetVoltage(1)
    time.sleep(0.5)
    for i in range(0,1023):
        set_val = i
        value = box.SetVoltage(set_val)
        pwm_values.append(set_val)
        voltage_values.append(int(value))

        #time.sleep(0.1)

    fig, ax1 = plt.subplots() 
    plt.title("PRS per PWM input")
    ax1.set_xlabel('PWM value') 
    ax1.set_ylabel('Voltage', color = 'red') 
    ax1.plot(pwm_values,voltage_values, color = 'red') 


    plt.show()
    


#main5()
#main3()
#main4()
##ls lg Q R
#ISystem_1(1,1,5,1)
#ISystem_scout_lg_ls()
#ISystem_scout_Q_R()

#FederMasse(1,1,1,3)
#BOX_SHOW_OUTPUT_CURVE()
AirFlow()
#SMDSystem_scout_lg_ls()
#SMDSystem_scout_Q_R()
#main2()
#Chessna()
#Chessna_2()
#FederMasse()
#FederMasse_MPC()
#FederMasse_PID()
#QuadCopter_system()
#InvertedPendulum_OnCart()
#PendulumWithCart_Animated()
#InvertedPendulum_MPC_WITHOUT_CART()
#BLDC_MOTOR_CONTROL()




