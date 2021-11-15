import sys
sys.path.append('../..')
sys.path.append('C:\\Users\\Martin\\Documents\\CARLA\\PythonAPI\\Master_Thesis_DEEPC_control')
import DeePC_control_2 as DeePCC2 # The code to test
import numpy as np
import carlaHelper as cHelper

import unittest   # The test framework
import logging # for printing extra info
data_path = ""

class Test_TestControllerInit(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.C2 = None
        self.log= logging.getLogger( "Test_TestControllerInit" )
    
    def setUp(self) -> None:
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.C2 = DeePCC2.Controller(self.data,5,5,3,3) # data, T_ini, T_f , nr inputs, nr outputs
        return super().setUp()

    def test_input_sequence(self):
        assert np.shape(self.C2.input_sequence) == (len(self.data),self.C2.input_size)

    def test_output_sequence(self):
        assert np.shape(self.C2.output_sequence) == (len(self.data),self.C2.output_size)
    
    def test_Hankel_matrix_input(self):
        U_p_f = self.C2.generateHankel_Collums(self.C2.L,self.C2.input_sequence)
        assert np.shape(U_p_f) == (self.C2.L * self.C2.input_size, len(self.C2.input_sequence) - self.C2.L )

    def test_Hankel_matrix_output(self):
        U_p_f = self.C2.generateHankel_Collums(self.C2.L,self.C2.output_sequence)
        assert np.shape(U_p_f) == (self.C2.L * self.C2.output_size, len(self.C2.output_sequence) - self.C2.L )

    def test_Hankel_numbersequence(self):
        testsequence = np.ones((50,5))
        test  = self.C2.generateHankel_Collums(7,testsequence)
        assert test[0][0] == 1

    def test_U_p_f_shape(self):
        #print(np.shape(self.C2.U_p_f)," == ",(self.C2.L * self.C2.output_size, len(self.C2.output_sequence) - self.C2.L))
        assert np.shape(self.C2.U_p_f) == (self.C2.L * self.C2.input_size, len(self.C2.input_sequence) - self.C2.L)
    
    def test_Y_p_f_shape(self):
        assert np.shape(self.C2.Y_p_f) == (self.C2.L*self.C2.output_size, len(self.C2.output_sequence) - self.C2.L)
    
    def test_U_p_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.U_p) == (self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L)
    
    def test_U_f_shape(self):
        assert np.shape(self.C2.U_f) == (self.C2.T_f * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L)

    def test_Y_p_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.Y_p) == (self.C2.T_ini * self.C2.output_size,len(self.C2.output_sequence) - self.C2.L)

    def test_Y_f_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.Y_f) == (self.C2.T_f * self.C2.output_size,len(self.C2.output_sequence) - self.C2.L)

class Test_TestControllerInit_other_dimenstions(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.data = np.asarray(self.data)
        self.data = self.data[:,:4]
        self.C2 = None
        self.log= logging.getLogger( "Test_TestControllerInit" )
    
    def setUp(self) -> None:
        self.C2 = DeePCC2.Controller(self.data,5,5,2,2) # data, T_ini, T_f , nr inputs, nr outputs
        return super().setUp()

    def test_input_sequence(self):
        assert np.shape(self.C2.input_sequence) == (len(self.data),self.C2.input_size)

    def test_output_sequence(self):
        assert np.shape(self.C2.output_sequence) == (len(self.data),self.C2.output_size)
    
    def test_Hankel_matrix_input(self):
        U_p_f = self.C2.generateHankel_Collums(self.C2.L,self.C2.input_sequence)
        assert np.shape(U_p_f) == (self.C2.L * self.C2.input_size, len(self.C2.input_sequence) - self.C2.L )

    def test_Hankel_matrix_output(self):
        U_p_f = self.C2.generateHankel_Collums(self.C2.L,self.C2.output_sequence)
        assert np.shape(U_p_f) == (self.C2.L * self.C2.output_size, len(self.C2.output_sequence) - self.C2.L )

    def test_Hankel_numbersequence(self):
        testsequence = np.ones((50,5))
        test  = self.C2.generateHankel_Collums(7,testsequence)
        assert test[0][0] == 1

    def test_U_p_f_shape(self):
        #print(np.shape(self.C2.U_p_f)," == ",(self.C2.L * self.C2.output_size, len(self.C2.output_sequence) - self.C2.L))
        assert np.shape(self.C2.U_p_f) == (self.C2.L * self.C2.input_size, len(self.C2.input_sequence) - self.C2.L)
    
    def test_Y_p_f_shape(self):
        assert np.shape(self.C2.Y_p_f) == (self.C2.L*self.C2.output_size, len(self.C2.output_sequence) - self.C2.L)
    
    def test_U_p_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.U_p) == (self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L)
    
    def test_U_f_shape(self):
        assert np.shape(self.C2.U_f) == (self.C2.T_f * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L)

    def test_Y_p_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.Y_p) == (self.C2.T_ini * self.C2.output_size,len(self.C2.output_sequence) - self.C2.L)

    def test_Y_f_shape(self):
        #print(np.shape(self.C2.U_p)," = ",(self.C2.T_ini * self.C2.input_size,len(self.C2.input_sequence) - self.C2.L))
        assert np.shape(self.C2.Y_f) == (self.C2.T_f * self.C2.output_size,len(self.C2.output_sequence) - self.C2.L)
    
class Test_TestControllerUpdate_U_Y_INI(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.data = np.asarray(self.data)
        self.data = self.data[:,:4]
        self.C2 = None
        self.log= logging.getLogger( "Test_TestControllerInit" )
    
    def setUp(self) -> None:
        print("Setting up")
        #self.data = cHelper.readFromCSV("sample_waypoints")
        self.C2 = DeePCC2.Controller(self.data,5,5,2,2) # data, T_ini, T_f , nr inputs, nr outputs
        return super().setUp()
        
    def test_shift_u_y_ini(self):
        self.C2.updateIn_Out_Measures([1,1],[2,2])
        self.C2.updateIn_Out_Measures([3,3],[4,4])
        self.C2.updateIn_Out_Measures([5,5],[6,6])
        self.C2.updateIn_Out_Measures([7,7],[8,8])
        self.C2.updateIn_Out_Measures([9,9],[10,10])
        self.C2.updateIn_Out_Measures([11,11],[12,12])
        print(np.shape(self.C2.u_ini))
        assert np.shape(self.C2.u_ini) == (self.C2.T_ini,self.C2.input_size)
        assert np.shape(self.C2.y_ini) == (self.C2.T_ini,self.C2.output_size)

class Test_TestControllerUpdate_U_Y_INI_other_shape(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.C2 = None
        self.log= logging.getLogger( "Test_TestControllerInit" )
    
    def setUp(self) -> None:
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.C2 = DeePCC2.Controller(self.data,5,5,3,3) # data, T_ini, T_f , nr inputs, nr outputs
        return super().setUp()
        
    def test_shift_u_y_ini(self):
        self.C2.updateIn_Out_Measures([1,1,1],[2,2,2])
        self.C2.updateIn_Out_Measures([3,3,3],[4,4,4])
        self.C2.updateIn_Out_Measures([5,5,5],[6,6,6])
        self.C2.updateIn_Out_Measures([7,7,7],[8,8,8])
        self.C2.updateIn_Out_Measures([9,9,9],[10,10,10])
        self.C2.updateIn_Out_Measures([11,11,11],[12,12,12])
        assert np.shape(self.C2.u_ini) == (self.C2.T_ini,self.C2.input_size)
        assert np.shape(self.C2.y_ini) == (self.C2.T_ini,self.C2.output_size)


if __name__ == '__main__':
    logging.basicConfig( stream=sys.stderr )
    logging.getLogger( "Test_TestControllerInit" ).setLevel( logging.DEBUG )
    unittest.main()
        
    
    