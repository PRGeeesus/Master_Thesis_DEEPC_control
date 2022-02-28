import DeePC_OSQP as C3
import random
import SimpleSystems
#from SimpleSystems import SimpleSystem1, FederMasseSystem
import matplotlib.pyplot as plt
import numpy as np

# test for Simplesystems:
    # SimpleSystem1()
    # FederMasseSystem()
    # myabe:
        # InvertedPendulimSS() ?
        # chessna ?
        # QuadCopter

# test sesitivity to T_d and T_ini
    # T_ini from 0 to 10 
    # T_d from moinimum to 350?

# test sensityvity of DEEPC osqp to lambda_s, lambda_g, q and p
    # l_g from 0 to 10^7
    # l_s from 0 to 10^10

# measue this way:
    # SUM ( || y(t) - y_r ||_2 ^2) for some experiemnt lendth
T_ini =3# über 3 = schwingen, unter 3 = linearer
T_f = 10

T_ini_range = range(0,10)
T_d_range_max   = 300
T_ini_max       = 15
#T_d_range_max   = 5*(1+1)*(T_f + T_ini_max)+1 # for T_ini measures be atleast sufficient

Controller_settings = {"lambda_s":20,
                        "lambda_g":10,
                        "out_constr_lb":[-10000],
                        "out_constr_ub":[10000],
                        "in_constr_lb":[-1],
                        "in_constr_ub":[1],
                        "regularize":True}


def testSensitivityto_Inputdata_length_I_system():
    print("Test Sensitivity to T_d of Linear Integrator System")

    # START COLLECTING DATA
    system1 = SimpleSystems.ISystem(0,0,1)
    for i in range(1,T_d_range_max):
        #system1.OneTick(input)
        system1.OneTick((random.random()-0.5)*2)
        system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
    outputs = system1.SystemHistory[:,1]
    #plt.plot(outputs)
    #plt.show()

    original_data = outputs

    tracking_error  = []
    x_values_t_d    = []
    print("Settings: " + str(Controller_settings))
    for TD_ref in range(((1+1)*(T_ini + T_f)+1),T_d_range_max):
        data = system1.SystemHistory[:TD_ref,:]
        print("Data length: " + str(len(data)) + " /" + str(T_d_range_max))

        ctrl = C3.Controller(data,T_ini,T_f,1,1,**Controller_settings)
        SOLLWERT = 15
        ctrl.updateReferenceWaypoint([SOLLWERT])
        #ctrl.updateReferenceInput([0.5])
    
        ctrl.updateControlCost_R([[1]]) # as I increase, only small changes in control
        ctrl.updateTrackingCost_Q([[1]]) #as I increase, prediction gets closer to SOLLWERT?
        #ctrl.updateReferenceInput([0.5])
        system1.resetSystem()
        predictions_y = [0]
        applied_inputs = []
        soll = [SOLLWERT]
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        prediction = 0
        control_input = 0
        for i in range(1,100):
            #if i == 60:
            #    ctrl.updateReferenceWaypoint([10])

            #ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
            if i > T_ini+3:
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
        track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(outputs2,predictions_y)
        control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,outputs2)

        tracking_error.append(control_mean)
        x_values_t_d.append(TD_ref)

    print("Experiment: Test Sensitivity to Inputdata length. Done. collected length:" + str(len(tracking_error)))
    fig, ax1 = plt.subplots()
    titlesting = "Sensitivity to T_d of Linear Integrator System"
    plt.title(titlesting)
    ax1.set_ylabel("tracking error")
    ax1.set_xlabel("data length")
    ax1.plot(x_values_t_d,tracking_error,label='tracking_error',c="b",marker = 'x')

    plt.legend(loc=8)
    plt.show()

# for some ready the Feder Masse System does not care how long the data that it gets is. It always controls the same
def testSensitivityto_Inputdata_length_FederMasse_system():
    print("Test Sensitivity to T_d of Feder Masse System")

    timesteps = 1
    sim_time = 200
    input_force = 5.0
    sys = SimpleSystems.FederMasseSystem(timesteps)
    timeline = [0.0]
    
    for i in range(T_d_range_max):
        if i%20 == 0:
            input_force = random.random()
        timeline.append(i*timesteps)
        sys.OneTick(input_force)


    inputs = sys.in_force
    x_out = sys.out_pos
    data = np.hstack([np.reshape(inputs,(len(inputs),1)),np.reshape(x_out,(len(x_out),1))])
    print("Init Data Length:" + str(len(data)))
    
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

    #### DEEPC PREDICTION PART
    tracking_error = []
    Data_length = []
    #T_d_range_min = ((1+1)*(T_ini + T_f)+8) #T >= (m+1)L-1 # where m = input size
    T_d_range_min = 34
    for TD_ref in range(T_d_range_min,T_d_range_max,5):
        data_cut = data[:TD_ref,:]
        print("Data Length: " +str(TD_ref)+" "+ str(len(data_cut)))

        ctrl = C3.Controller(data_cut,T_ini,T_f,1,1,**Controller_settings)
        SOLLWERT = 5
        ctrl.updateReferenceWaypoint([SOLLWERT])
        #ctrl.updateReferenceInput([0.5])
        ctrl.updateControlCost_R([[1]])
        ctrl.updateTrackingCost_Q([[1]])
        sys.resetSystem()

        predictions_y = [0]
        applied_inputs = [0]
        soll = [SOLLWERT]
        timeline = [0]
        #pid = PID(0.9, 10, 0.1, setpoint=SOLLWERT)
        #pid.output_limits = (-5.0, 5.0)
        #pid.sample_time = 0.005
        Control_input = 0
        prediction = 0
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        for i in range(0,100): 
            timeline.append(i)
            if i > T_f +3:
                u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
                Control_input = u[0][0]
                prediction = y_star[0][0]
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

        tracking_error.append(control_mean)
        Data_length.append(TD_ref)


    fig, ax1 = plt.subplots()
    titlesting = "DeePC control of Spring Mass System dependent on Data Length"
    plt.title(titlesting)
    ax1.set_ylabel("tracking error")
    #ax1.plot(data[:,1],label='Init Data')
    #ax1.plot(timeline,x_out,label='Init Data',c='r')
    ax1.plot(Data_length,tracking_error,label='tracking_error',c="b")
    plt.legend(loc=8)
    plt.show()
    SimpleSystems.SaveSystemSettings("DeePC-Spring-Mass-on-Data-Length__LOG",sys,ctrl)

def testSensitivityto_T_ini_I_system():
    print("Test Sensitivity to T_ini of Linear Integrator System")

    # START COLLECTING DATA
    system1 = SimpleSystems.ISystem(0,0,1)
    for i in range(1,T_d_range_max):
        #system1.OneTick(input)
        system1.OneTick((random.random()-0.5)*2)
        system1.OneTick(random.random())
        #system1.OneTick(np.abs(np.sin(i*0.1)))
    outputs = system1.SystemHistory[:,1]
    #plt.plot(outputs)
    #plt.show()

    original_data = outputs

    tracking_error  = []
    x_values_T_ini    = []
    print("Settings: " + str(Controller_settings))
    for T_ini_value in range(1,T_ini_max):
        data = system1.SystemHistory[:T_d_range_max-1,:]
        print("T_ini_value: " + str(T_ini_value))

        ctrl = C3.Controller(data,T_ini_value,T_f,1,1,**Controller_settings)
        SOLLWERT = 15
        ctrl.updateReferenceWaypoint([SOLLWERT])
        #ctrl.updateReferenceInput([0.5])
    
        ctrl.updateControlCost_R([[1]]) # as I increase, only small changes in control
        ctrl.updateTrackingCost_Q([[1]]) #as I increase, prediction gets closer to SOLLWERT?
        #ctrl.updateReferenceInput([0.5])
        system1.resetSystem()
        predictions_y = [0]
        applied_inputs = []
        soll = [SOLLWERT]
        u,y,u_star,y_star,g = ctrl.getInputOutputPrediction()
        prediction = 0
        control_input = 0
        for i in range(1,100):
            #if i == 60:
            #    ctrl.updateReferenceWaypoint([10])

            #ctrl.updateReferenceWaypoint([10*np.sin((np.pi*2*(1/200)*i))+10])
            if i > T_ini+3:
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
        track_mean,track_std = SimpleSystems.Evaluate_Tracking_Accuarcy(outputs2,predictions_y)
        control_mean,control_std = SimpleSystems.Evaluate_Control_Accuarcy(soll,outputs2)

        tracking_error.append(control_mean)
        x_values_T_ini.append(T_ini_value)

    print("Experiment: Test Sensitivity to Inputdata length. Done. collected length:" + str(len(tracking_error)))
    fig, ax1 = plt.subplots()
    titlesting = "Sensitivity to T_d of Linear Integrator System"
    plt.title(titlesting)
    ax1.set_ylabel("tracking error")
    ax1.set_xlabel("data length")
    ax1.plot(x_values_T_ini,tracking_error,label='tracking_error',c="b",marker = 'x')

    plt.legend(loc=8)
    plt.show()
    # Save Image and Information about Experiement
    experiment_name = "DeePC-I-System-on-T_ini-Length"
    plt.savefig(experiment_name + "__LOG""__FIGUTE.png")
    SimpleSystems.SaveSystemSettings(experiment_name + "__LOG",sys,ctrl)

def calculateTrackingError(output_array, reference_array):
    if( len(output_array) != len(reference_array)):
        print("Output and references not same length")
        return 0
    summation = 0
    for i in range(len(output_array)):
        summation = summation + np.sqrt(np.square(output_array[i]) - np.square(reference_array[i]))

    return summation

def main():
    #testSensitivityto_Inputdata_length()
    #testSensitivityto_Inputdata_length_FederMasse_system()
    testSensitivityto_T_ini_I_system()

if __name__ == '__main__':
    
    main()