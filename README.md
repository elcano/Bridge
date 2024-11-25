# Bridge
A circuit board that ties together three Arduino Dues.

The Elcano Simulator Bridge printed circuit board (pcb) was designed in 2020. It ties together three Arduino Dues to perform a single task. Documentation is on https://github.com/elcano/Bridge. The board can be used to run three Arduino Dues together, partitioning tasks to different boards.
The interconnections are given in Due CARLA_Bridge_Pin_Mappings.xlsx.  The three Arduino Dues are labeled
1.	LL (Low Level) or DBW (Drive by wire)
2.	HL (High Level) or SH (Sensor Hub)
3.	Router
Each board takes the CAN RX and TX signals to a 2561 chip that converts them to CAN HI and Lo.
The router is connected to an SD card.
The board can take external DC power and converts it to the 3.3 V and 5V supplies used by the components.
The file BridgeSchematic.png gives a schematic.
BOM_V4_CARLA_Bridge.xlsx is the bill of materials for the components on the bridge board.
In its original function, DBW controlled the throttle, brakes and steering for an autonomous vehicle. SH provided GPS, Magnetometer (digital compass), Sonar readings (C5_Rear and C5_Front), Speedometer (Wheel_Rotation), Optical 2D speedometer (Mouse) and Gyroscope. Additional sensor information would be available over the CAN bus.  The Router board connected DBW and SH to a PC running a simulator. The output from DBW went to the simulated vehicle and the simulated state of the vehicle was the input to the sensors on SH. The simulator bridge allowed one to test the hardware and software controlling a vehicle on a simulator.
