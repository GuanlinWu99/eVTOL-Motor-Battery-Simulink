1. Battery and Motor integration
Uav_Electrical_lib.slx is based on HEV_Electrical_Lib.slx, adding four motors. 
HEV_Electrical_Lib.slx build the relationships between Motor and Battery.

2. Battery
HEV_Battery_Lib.slx build the battery pack, and Battery_Cell_Det.slx build the battery cell, and Rcecm_lib.slx build the 2RC circuit components.

3. Testing
Test_FourMotor.slx could test four motors, and Test_FourMotor_PID.slx could test four motor using motor controller.


Battery_Cell_Det.slx   ──►  HEV_Battery_Lib.slx
                              │
                              ▼
                        HEV_Electrical_Lib.slx
                              │
           ┌──────────────────┴───────────────────┐
           ▼                                      ▼
  Uav_Electrical_Lib.slx              Test_FourMotor(.slx / _PID.slx)


How to Run:
Intergation: open prj, then run the src/uav_param first, then run startup.m.
Four motor testing: Run src/uav_param.m, then run Test_FourMotor.slx
Four motor with PID testing: Run src/uav_param.m, then run Test_FourMotor_PID.slx.