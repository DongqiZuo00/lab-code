#Here we define our own apps, is included by the file "nuttx_crazyflie_default.cmake"

set(config_module_list
    ${config_module_list}
    Apps/dw1000ranging
    Apps/QuadcopterLogic
    Apps/cf_motors
    Apps/esc_motors
    Apps/CalibrateIMU
    Apps/FlightRecorder
    Apps/USBDataOut
    
		# lib/Common
		# lib/Components
    )
    
