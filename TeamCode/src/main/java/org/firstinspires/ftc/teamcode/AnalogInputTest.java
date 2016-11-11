package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by Daniel on 11/11/2016.
 */

public class AnalogInputTest extends OpMode{

        AnalogInput analog; // object variable.
public void init() // Automatically called at program started
        {
        analog = hardwareMap.analogInput.get("ods"); // Create the analog object
        }
public void start() // Automatically called at program started
        { // Nothing to do for now.
        }
public void loop() // Looping code
        {
        double test = analog.getVoltage(); // Get analog value (0-1023).
                telemetry.addData("Daniel and Hirsh do nothing", "-FTC Team");
            telemetry.addData("How far away stupid stuff is:", String.format("%4d", test)); // print to screen
                telemetry.update();
        }
public void stop() // Automatically called at end of teleop.
        { // Nothing to do for now.
        }
        } // End of the demo class