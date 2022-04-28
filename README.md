
Implementation of DeePC

This repository contains the files used in the implementation of the DeePC algorithms.
It also contrains all python scrips used to conduct any experiment.

### Important files

- DeePC_OSQP.py          - This file contains the main implementation of the controller class.
- SimpleSystems.py       - This file contains the Classes for each simulated system and any additional 
		     	   Things that were simulated
- main_simple_system.py  - This file combines the controller with the simulated systems.
- Voltage_Controller.py  - This file contrains the hardware interface for the Voltage controller / hardware interface.


### Folders

- documentation 		- Here you can find a detailed documentation fo the fileds DeePC.py and SimpleSystems.py
				since they are .html file they are best downloaded and viewed in the browser, since gitHub will
				only show the code.
- StoreData     		- All .csv files are stored away here.
- Arduino_Voltage_Controller    - The firmware for the Arduino is in here


### Notice
Ignore any files that referr to Carla or vehicle. This was a try to apply DeePC to a vehicle simulator (CARLA) by microsoft, 
which was abbandoned afer many tries.






