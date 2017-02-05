package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Locale;


// * Created by definitly not HIRSH as he would mess it up and it would explode on 8/18/2016.

@TeleOp(name= "HDriveTeleop Field Centric")
public class HDriveTeleop extends OpMode {
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    Orientation angles;
    BNO055IMU imu;
    String angleDouble = "hi";
    public double gyroAngle;
    boolean speedMode = false;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotor shooter;
    boolean countUp = false;
    int countsinceapressed = 0;
    boolean countUp2 = false;
    int countsincebpressed = 0;
    //HDrive2 calculator;
    HDriveFCCalc calculator;
    Servo servo2;
    double armAngle = .5;
    double offset = 0;
    int encoder = 0;
    int shootTimer = 6;
    boolean bumperPressed = false;
    boolean bumperIsPressed = false;
    boolean hulianNotAnH = false;
    boolean shootTimerDone = false;
    Servo buttonPusher;
    boolean lezGoSlow = false;
    boolean state1;
    Servo IntakeServo;
    DcMotor IntakeMotor;
    public HDriveTeleop(){

    }
    public void init(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        shooter = hardwareMap.dcMotor.get("shooter");
        //servo2 = hardwareMap.servo.get("servo2");
        buttonPusher = hardwareMap.servo.get("servo2");



        calculator = new HDriveFCCalc();
        //servo = hardwareMap.crservo.get("servo");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        int i = 0;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        float left = gamepad1.left_trigger;
        float right = gamepad1.right_trigger;
        float leftTrigger = gamepad2.left_trigger;
        float rightTrigger = gamepad2.right_trigger;
        boolean buttonAPressed = gamepad1.a;
        boolean buttonXPressed = gamepad1.x;
        boolean buttonAPressed2 = gamepad2.a;
        boolean buttonXPressed2 = gamepad2.x;
        boolean dPadUp    = gamepad1.dpad_up;
        boolean dPadDown  = gamepad1.dpad_down;
        boolean dPadLeft  = gamepad1.dpad_left;
        boolean dPadRight = gamepad1.dpad_right;
        //bumperPressed = gamepad1.right_bumper;
        if(countUp){
            if(countsinceapressed < 10){
                countsinceapressed++;
            }
            else{
                countUp = false;
                countsinceapressed = 0;
            }
        }
        if(buttonAPressed&& !countUp) {
            countUp = true;
            if (speedMode == false) {
                speedMode = true;
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Mode", "Speed");
                telemetry.update();
            } else if (speedMode == true) {
                speedMode = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Mode", "Power");
                telemetry.update();
            }
        }
        // telemetry.addData("Encoder Position", shooter.getCurrentPosition());
        // telemetry.addData("Angle", Double.parseDouble(angleDouble)+offset);
        // telemetry.addData("Left Trigger", gamepad1.left_trigger);
        // telemetry.addData("Left Stick X" , gamepad1.left_stick_x);
        //telemetry.addData("Right Stick Y" , gamepad1.right_stick_x);
        //telemetry.addData("Left Stick X" , gamepad1.left_stick_y);
        //telemetry.addData("Right Stick Y" , gamepad1.right_stick_y);
        //telemetry.update();
        if(gamepad1.y) {
            lezGoSlow = true;
        }
        else {
            lezGoSlow = false;
        }

        if(buttonXPressed == true){
            offset = Double.parseDouble(angleDouble);
            offset = -offset;
        }
        calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + offset);
        if(lezGoSlow) {
            if (!speedMode) {
                leftMotor.setPower(.1 * calculator.getLeftDrive());
                rightMotor.setPower(.1 * calculator.getRightDrive());
                middleMotor.setPower(.2*-calculator.getMiddleDrive());
            } else {
                leftMotor.setPower(.1*calculator.getLeftDrive());
                rightMotor.setPower(.1*calculator.getRightDrive());
                middleMotor.setPower(.2*-calculator.getMiddleDrive());
            }
        }
        else {
            if (!speedMode) {
                leftMotor.setPower(.7 * calculator.getLeftDrive());
                rightMotor.setPower(.7 * calculator.getRightDrive());
                middleMotor.setPower(-calculator.getMiddleDrive());
            } else {
                leftMotor.setPower(calculator.getLeftDrive());
                rightMotor.setPower(calculator.getRightDrive());
                middleMotor.setPower(-calculator.getMiddleDrive());
            }
        }
        if(left > 0) {
            buttonPusher.setPosition(1);
        }
        if(right > 0) {
            buttonPusher.setPosition(0);
        }
   /*   if(gamepad1.left_bumper == true) {
          leftMotor.setPower(-.1);
          rightMotor.setPower(-.1);
      }
      if(gamepad1.right_bumper == true) {
          leftMotor.setPower(.1);
          rightMotor.setPower(.1);
<<<<<<< HEAD
      }*/
        if(!shootTimerDone){
            shootTimer++;
            if(shootTimer > 5){
                shootTimerDone = true;
            }
        }
        if(gamepad1.left_bumper && shootTimerDone && !bumperIsPressed) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setTargetPosition(3360);
            shooter.setPower(1);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bumperIsPressed = true;
            //telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
            //telemetry.update();
            shootTimer = 0;
        }
        if(shooter.getCurrentPosition() >= 3359 && shooter.getCurrentPosition() <= 3365 && bumperIsPressed) {
            shootTimerDone = false;
            shooter.setPower(0);
            bumperIsPressed = false;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
            //telemetry.update();
        }
        //telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
        //telemetry.update();
      /*if(gamepad1.right_bumper) {
          shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          shooter.setTargetPosition(3120);
          shooter.setPower(1);
          shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          hulianNotAnH = true;
      }
      //set to maximum overdrive
      if(shooter.getCurrentPosition() == 3120 && hulianNotAnH == true) {
          shooter.setPower(0);
          hulianNotAnH = false;
          shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
      servo2.setPosition(armAngle);
      */
        //****************************************************************
        IntakeServo= hardwareMap.servo.get("IntakeServo");
        if (gamepad2.right_bumper == true){
            IntakeServo.setPosition(1);
        }
        else {IntakeServo.setPosition(0);
        }
        IntakeMotor= hardwareMap.dcMotor.get("IntakeMotor");
        if (gamepad2.left_bumper == true){
            IntakeMotor.setPower(1);
        }
        else {IntakeMotor.setPower(0);
        }


        if(gamepad1.right_bumper) {
            bumperPressed = true;
        }
        else {
            bumperPressed = false;
        }
        telemetry.addData("Gyro",Double.parseDouble(angleDouble) + offset);
        telemetry.update();
        //***********************************************************************
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));


    }
}


