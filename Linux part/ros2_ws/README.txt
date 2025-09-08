In order to physicaly launch the nutrunner :

In terminal :
pip install pymodbus

~/ros2_ws/src/modbus_interface/modbus_interface/modbus_node.py  #is to be replaced by your files


		## Setup.py : add :

		entry_points={
		    'console_scripts': [
			'modbus_node = modbus_interface.modbus_node:main',
		    ],
		},

Then in terminal :

cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select modbus_interface
source install/setup.bash


ros2 run modbus_interface modbus_node --ros-args -p coil_address:=X ##label of the output you wanna manage: from 0 to 2 binary code for cycle selection; 3 is start cycle.
 
Then in another terminal :

ros2 service call /set_coil std_srvs/srv/SetBool "{data: true}"

So, for example, to start cycle number 1, you need to run the following command:


	Terminal 1 : 

	ros2 run modbus_interface modbus_node --ros-args -p coil_address:=0 ##

	Terminal 2 : 

	ros2 service call /set_coil std_srvs/srv/SetBool "{data: true}"

	Terminal 1 : 

	#*ctrl+c*
	ros2 run modbus_interface modbus_node --ros-args -p coil_address:=3 ##

	Terminal 2 : 

	#*ctrl+c*
	ros2 service call /set_coil std_srvs/srv/SetBool "{data: true}"


And to end everything :

	Terminal 1 : 

	ros2 run modbus_interface modbus_node --ros-args -p coil_address:=0 ##

	Terminal 2 : 

	ros2 service call /set_coil std_srvs/srv/SetBool "{data: false}"

	Terminal 1 : 

	#*ctrl+c*
	ros2 run modbus_interface modbus_node --ros-args -p coil_address:=3 ##

	Terminal 2 : 

	#*ctrl+c*
	ros2 service call /set_coil std_srvs/srv/SetBool "{data: false}"


