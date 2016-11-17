package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * Created by HIRSH on 8/18/2016.
 */




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
* Created by Luke on 8/18/2016.
*/

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// * Created by Hirsh on 10/23/2016.
@Autonomous(name= "Servo")
public class HirshIsCoolTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = this.hardwareMap.crservo.get("servo2");
        waitForStart();
        servo.setPower(.2);
        Thread.sleep(100);

    }
}