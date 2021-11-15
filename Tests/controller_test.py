import sys
sys.path.append('../..')
sys.path.append('C:\\Users\\Martin\\Documents\\CARLA\\PythonAPI\\Master_Thesis_DEEPC_control')
import DeePC_control as DeePCC # The code to test
import numpy as np
import carlaHelper as cHelper

import unittest   # The test framework
import logging # for printing extra info
data_path = ""

class Test_TestControllerInit(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName=methodName)
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.CONTROLLER = None
        self.log= logging.getLogger( "Test_TestControllerInit" )
    def setUp(self) -> None:

        return super().setUp()

    def test_with_3_in_3_outputs(self):
        self.data = cHelper.readFromCSV("sample_waypoints")
        self.CONTROLLER = DeePCC.Controller(self.data,5,6,3,3) # data, T_ini, T_f , nr inputs, nr outputs
        self.importData()
        self.shape_DataArrays_U_p_and_f()
    
    # def test_with_3_in_2_outputs(self):
    #     self.data = cHelper.readFromCSV("sample_waypoints")
    #     self.data = np.array(self.data)
    #     self.data = self.data[:,:5]

    #     self.CONTROLLER = DeePCC.Controller(self.data,5,6,3,2) # data, T_ini, T_f , nr inputs, nr outputs
    #     self.importData()
    #     self.shape_DataArrays_U_p_and_f()
    
    def test_with_all_ones_data(self):
        data_length = 50
        self.data = [[1.0,1.0,1.0,1.0,1.0,1.0] for i in range(data_length)]
        self.CONTROLLER = DeePCC.Controller(self.data,5,5,3,3) # data, T_ini, T_f , nr inputs, nr outputs
        testinput = self.CONTROLLER.input_sequence[:12]
        testoutput = self.CONTROLLER.output_sequence[:12]
        for i in range(len(testinput)):
            self.CONTROLLER.updateInputOutputMeasures(testinput[i],testoutput[i])
        reference = [1.0,1.0,1.0]
        self.CONTROLLER.updateReferenceWaypoint(reference)
        self.log.debug(np.shape(self.CONTROLLER.u_ini))
        self.log.debug(np.shape(self.CONTROLLER.y_ini))
        u,y,g = self.CONTROLLER.find_g_star()
        



    #c = Controller()
    def importData(self):
        self.assertIsNotNone(self.data)

    def shape_DataArrays_U_p_and_f(self):
        DATA_SIZE  = np.shape(self.data)
        assert np.shape(self.CONTROLLER.U_p_f) == (self.CONTROLLER.L * self.CONTROLLER.output_size,len(self.CONTROLLER.output_sequence)-self.CONTROLLER.L)
        assert np.shape(self.CONTROLLER.Y_p_f) == (self.CONTROLLER.L * self.CONTROLLER.output_size,len(self.CONTROLLER.output_sequence)-self.CONTROLLER.L)
        assert np.shape(self.CONTROLLER.U_p) == (self.CONTROLLER.T_ini * self.CONTROLLER.input_size,len(self.CONTROLLER.input_sequence)-self.CONTROLLER.L)
        assert np.shape(self.CONTROLLER.U_f) == (self.CONTROLLER.T_f * self.CONTROLLER.input_size,len(self.CONTROLLER.input_sequence)-self.CONTROLLER.L)

        assert np.shape(self.CONTROLLER.Y_p) == (self.CONTROLLER.T_ini * self.CONTROLLER.output_size,len(self.CONTROLLER.output_sequence)-self.CONTROLLER.L)
        assert np.shape(self.CONTROLLER.Y_f) == (self.CONTROLLER.T_f * self.CONTROLLER.output_size,len(self.CONTROLLER.output_sequence)-self.CONTROLLER.L)
    
    # def test_logginData(self):
    #     log= logging.getLogger( "Test_TestControllerInit" )# this is defiend in main
    #     log.debug( "blabla = %r", 3.00) # pass argunents like this
    #     assert True



if __name__ == '__main__':
    logging.basicConfig( stream=sys.stderr )
    logging.getLogger( "Test_TestControllerInit" ).setLevel( logging.DEBUG )
    unittest.main()