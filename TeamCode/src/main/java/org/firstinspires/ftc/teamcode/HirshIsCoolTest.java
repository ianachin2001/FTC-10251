/*package org.firstinspires.ftc.teamcode;
import android.util.Log;

/*import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
<<<<<<< HEAD
=======
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.CRServo;
<<<<<<< HEAD


/**
 * Created by HIRSH on 8/18/2016.


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name= "HDriveTeleop")
// * Created by Hirsh on 10/23/2016.
@TeleOp(name= "HDrive: Servo Test")
public class HirshIsCoolTest extends OpMode {
    CRServo servo2;

    public void init(){
        servo2 = hardwareMap.crservo.get("servo2");
        //CRServo servo = this.hardwareMap.crservo.get("servo");
        try {
            servo2.setPower(1.0);
            servo2.wait(4000);
            servo2.setPower(0.0);
            servo2.wait(4000);
            servo2.setPower(-1.0);
            servo2.wait(4000);
        }
        catch(InterruptedException e){
            e.printStackTrace();
        }
    }
    public void loop(){

    }
}


/*=======
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
* Created by Luke on 8/18/2016.
[

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// * Created by Hirsh on 10/23/2016.
@Autonomous(name= "Servo")
public class HirshIsCoolTest extends LinearOpMode {
    @Override
        public void runOpMode() throws InterruptedException{
        CRServo servo2 = this.hardwareMap.crservo.get("servo2");
            waitForStart();
            servo2.setPower(.2);
            Thread.sleep(1143);
           /* servo2.setPower(0.0);
            Thread.sleep(4000);

    }

}
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d*/
