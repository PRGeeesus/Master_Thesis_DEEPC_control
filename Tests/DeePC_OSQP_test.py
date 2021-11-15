import sys
sys.path.append('../..')
sys.path.append('C:\\Users\\Martin\\Documents\\CARLA\\PythonAPI\\Master_Thesis_DEEPC_control')
import DeePC_control as DeePCC # The code to test
import DeePC_OSQP 
import numpy as np
import carlaHelper as cHelper

import unittest   # The test framework
import logging # for printing extra info
data_path = ""

class Test_Object_Creation(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("TestData")
        self.DeeC = DeePC_OSQP.Controller(self.data, 6, 6, 3, 3)
        self.log= logging.getLogger( "Test_TestControllerInit" )
        
    def setUp(self) -> None:
        return super().setUp()

    def test_input_sequence_shape(self):
        shape1 = np.shape(self.DeeC.input_sequence)
        assert shape1 == (len(self.data),self.DeeC.input_size)
    
    def test_input_sequence_shape(self):
        shape1 = np.shape(self.DeeC.output_sequence)
        assert shape1 == (len(self.data),self.DeeC.output_size)

    def test_U_p_f_shape(self):
        shape1 = np.shape(self.DeeC.U_p_f)
        shape2 = (((self.DeeC.T_f + self.DeeC.T_ini)*self.DeeC.input_size),(len(self.data) - self.DeeC.L))
        assert shape1 == shape2
    
    def test_U_p_f_elements(self):
        array = self.DeeC.U_p_f
        assert array[0][0] == 0.1
        assert array[1][0] == 0.2
        assert array[2][0] == 0.3

        assert array[3][0] == 1.1
        assert array[4][0] == 1.2
        assert array[5][0] == 1.3

        assert array[-1][0] == self.DeeC.L + 0.3 - 1
        assert array[-1][1] == self.DeeC.L + 0.3

        assert array[-1][-1] == self.data[-1][2]-1
    
    def test_updateReferenceWaypoint(self):
        new_reference = [12,12,12]
        self.DeeC.updateReferenceWaypoint(new_reference)
        assert self.DeeC.y_r == new_reference
    
    def test_update_in_out_measures(self):
        new_in  = [12,12,12]
        new_out = [12,12,12]
        self.DeeC.updateIn_Out_Measures(new_in,new_out)
        assert self.DeeC.u_ini[-1][0] == new_in[0]
        assert self.DeeC.y_ini[-1][0] == new_out[0]

    def test_update_in_out_to_correct_length(self):
        new_in  = [12,12,12]
        new_out = [12,12,12]
        for i in range(self.DeeC.T_ini + 2):
            self.DeeC.updateIn_Out_Measures(new_in,new_out)

        assert len(self.DeeC.u_ini) == self.DeeC.T_ini
        assert len(self.DeeC.y_ini) == self.DeeC.T_ini

    
    def test_gr_shape(self):
        shape1 = np.shape(self.DeeC.g_r)
        #shape2 = (self.DeeC.T_f*2*self.DeeC.input_size + self.DeeC.T_ini*2*self.DeeC.output_size,1)
        #shape3 = (self.DeeC.T_f*2*self.DeeC.input_size + self.DeeC.T_ini*2*self.DeeC.output_size,(T-L))
        shape2 = (self.DeeC.data_length - self.DeeC.L,)
        assert shape1 == shape2

    def test_gr_elements(self):
        shape1 = np.shape(self.DeeC.g_r)
        #shape2 = (self.DeeC.T_f*2*self.DeeC.input_size + self.DeeC.T_ini*2*self.DeeC.output_size,1)
        #shape3 = (self.DeeC.T_f*2*self.DeeC.input_size + self.DeeC.T_ini*2*self.DeeC.output_size,(T-L))
        shape2 = (self.DeeC.data_length - self.DeeC.L,)
        assert shape1 == shape2




if __name__ == '__main__':
    logging.basicConfig( stream=sys.stderr )
    logging.getLogger( "Test_TestControllerInit" ).setLevel( logging.DEBUG )
    unittest.main()