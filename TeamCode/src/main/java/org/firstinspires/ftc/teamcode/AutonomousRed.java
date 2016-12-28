package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by Luke on 9/25/2016.
 */
@TeleOp(name= "Red Autonomous", group = "HDrive")
public class AutonomousRed extends LinearOpMode {

    Orientation angles;
    BNO055IMU imu;
    OpticalDistanceSensor opticalDistanceSensor;
    AnalogInput distanceSensor;
    private ElapsedTime runtime = new ElapsedTime();
    //HardwareHDrive robot   = new HardwareHDrive();   // Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1040 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    TouchSensor             touchsensor;
    ColorSensor             sensorRGB;
    String angleDouble;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotorController controller;
    DcMotorController controller2;
    ServoController controller3;
    Double gyroDouble;
    double initialAngle;
    Servo servo2;

    public void runOpMode() throws InterruptedException {
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        int timeStart;
        int timeEnd;
        int encodersRunning;


        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        touchsensor = hardwareMap.touchSensor.get("touch_sensor");
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

        //robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        controller = hardwareMap.dcMotorController.get("Motor Controller 1");
        controller2 = hardwareMap.dcMotorController.get("Motor Controller 2");
        controller3 = hardwareMap.servoController.get("Servo Controller 1");
        distanceSensor = hardwareMap.analogInput.get("ODS");
        servo2 = hardwareMap.servo.get("servo2");
        sensorRGB = hardwareMap.colorSensor.get("color");

            /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        encoderDrive(0.7, -43,-43, 100.0); //Drive let and right motors backwards 24-17 inches
        Thread.sleep(300);
        encoderDriveMiddle(.3,43,20.0);

        leftMotor.setPower(0); //Stop the left Motor
        rightMotor.setPower(0); //Stop the Right Motor
        middleMotor.setPower(0);
        middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleMotor.setPower(-.1);
        int x = 0;
        while(x < 3) {
            x++;
            Thread.sleep(500);
        }
        middleMotor.setPower(0);
        /*leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(500);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setPower(-.03);
        timeStart = Math.abs(middleMotor.getCurrentPosition());
        Thread.sleep(500);
        timeEnd = Math.abs(middleMotor.getCurrentPosition());
        encodersRunning = timeEnd - timeStart;
        while(encodersRunning > 40){
            timeStart = Math.abs(middleMotor.getCurrentPosition());
            Thread.sleep(500);
            timeEnd = Math.abs(middleMotor.getCurrentPosition());
            encodersRunning = timeEnd - timeStart;
            telemetry.addData("Encoders Per 100 Seconds", encodersRunning);
            telemetry.update();
        }
        middleMotor.setPower(0);
        telemetry.addData("Fin", "Fin");
        telemetry.update();*/
        middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(300);
        servo2.setPosition(1);
        middleMotor.setPower(-.1);
        while(distanceSensor.getVoltage() < .7) {
            leftMotor.setPower(-.1);
            rightMotor.setPower(-.1);
            telemetry.addData("Distance Sensor", distanceSensor.getVoltage());
            telemetry.update();
        }
        Thread.sleep(100);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        middleMotor.setPower(0);
        Thread.sleep(100);
        if(sensorRGB.red() > sensorRGB.blue()) {
            telemetry.addData("Color", "Red");
            middleMotor.setPower(-.1);
            Thread.sleep(1400);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderDrive(-.1,-5,-5,20);
            servo2.setPosition(.5);
            Thread.sleep(100);
            servo2.setPosition(1);
            Thread.sleep(800);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderDrive(-.2,-8,-8,20);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            servo2.setPosition(1);


            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setPower(-.1);
            encoderDrive(.6, -21, -21, 20.0);
            Thread.sleep(100);
            middleMotor.setPower(0);
        }
        else if(sensorRGB.blue() > sensorRGB.red()) {
            telemetry.addData("Color", "Blue");
            Thread.sleep(100);
            servo2.setPosition(.5);
            Thread.sleep(100);
            servo2.setPosition(1);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderDrive(-.1,-10,-10,20);
            middleMotor.setPower(-.1);
            Thread.sleep(1400);
            middleMotor.setPower(0);
            servo2.setPosition(1);


            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setPower(-.1);
            encoderDrive(.6, -26, -26, 20.0);
            Thread.sleep(100);
            middleMotor.setPower(0);

        }
        else {
            telemetry.addData("You are terrible at programming", "-Robot");
        }

        //middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servo2.setPosition(1);
        Thread.sleep(100);
        middleMotor.setPower(0);
        while(distanceSensor.getVoltage() < .7) {
            leftMotor.setPower(-.2);
            rightMotor.setPower(-.2);
            middleMotor.setPower(-.1);
        }
        Thread.sleep(250);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        middleMotor.setPower(0);
        Thread.sleep(100);
        if(sensorRGB.red() > sensorRGB.blue()) {
            telemetry.addData("Color", "Red");
            leftMotor.setPower(-.4);
            rightMotor.setPower(-.4);
            middleMotor.setPower(-.1);
            Thread.sleep(300);
            servo2.setPosition(.5);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            Thread.sleep(100);
            leftMotor.setPower(-.1);
            rightMotor.setPower(-.1);
            middleMotor.setPower(-.1);
            Thread.sleep(2000);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            Thread.sleep(250);
            servo2.setPosition(1);
        }
        else if(sensorRGB.blue() > sensorRGB.red()) {
            telemetry.addData("Color", "Blue");
            Thread.sleep(100);
            servo2.setPosition(.5);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(-.1);
            Thread.sleep(200);
            leftMotor.setPower(-.1);
            rightMotor.setPower(-.1);
            middleMotor.setPower(-.1);
            Thread.sleep(1300);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            Thread.sleep(50);
            servo2.setPosition(1);
        }
        else {
            telemetry.addData("You are terrible at programming", "-Robot");
        }
        servo2.setPosition(1);


        /*angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);//Initialize gyro
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle); //Initialize Gyro
        initialAngle = Double.parseDouble(angleDouble); //Set the Gyro angle to Double (Originally String)
        Thread.sleep(500); //Wait .5 seconds
        while(initialAngle<86.5) { // Subtract 3.5 degrees because the while has a delay or something
            rightMotor.setPower(.1);
            leftMotor.setPower(-.1);
            telemetry.clearAll();
            telemetry.addData("Gyro Angle" , angleDouble);
            telemetry.update();
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
            initialAngle = Double.parseDouble(angleDouble);
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        telemetry.clearAll();
        telemetry.addData("Turned 90" , "Degrees");
        telemetry.update();*/




        /*angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        initialAngle = Double.parseDouble(angleDouble);
        telemetry.clearAll();
        telemetry.addData("Gyro Angle", angleDouble);
        telemetry.update();*/

        //encoderDrive(DRIVE_SPEED, -12, -12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDriveMiddle(DRIVE_SPEED, 12, 5.0);






    }



    public void encoderDriveMiddle(double speed,//int leftDistance, int rightDistance,
                                   double middleInches,
                                   double timeoutS) throws InterruptedException {

        int newMiddleTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches + 9) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                telemetry.addData("Path2", "Running at %7d",

                        middleMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double speed,//int leftDistance, int rightDistance,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {



        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());

                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

