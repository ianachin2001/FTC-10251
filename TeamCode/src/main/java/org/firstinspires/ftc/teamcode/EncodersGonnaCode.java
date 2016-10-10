package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Luke on 9/25/2016.
 */
@TeleOp(name= "EncodersGonnaCode", group = "HDrive"
)
public class EncodersGonnaCode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareHDrive robot   = new HardwareHDrive();   // Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotorController controller;

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        controller = hardwareMap.dcMotorController.get("Motor Controller 1");
            /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition(),
                robot.middleMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();


        //encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
        //encoderDrive(DRIVE_SPEED, -12, -12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDriveMiddle(DRIVE_SPEED, 12, 5.0);




    }

    public void encoderDriveMiddle(double speed,double middleInches,
                             double timeoutS) throws InterruptedException {



        int newMiddleTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = robot.middleMotor.getCurrentPosition() + (int)(middleInches * COUNTS_PER_INCH);

            robot.middleMotor.setTargetPosition(newMiddleTarget);


            // Turn On RUN_TO_POSITION
            robot.middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.middleMotor.setPower(Math.abs(speed));
            // robot.leftMotor.setPower((speed));
            //robot.rightMotor.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.middleMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newMiddleTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.middleMotor.getCurrentPosition());

                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.middleMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
   /* public void encoderDrive(double speed,//int leftDistance, int rightDistance,
                           double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {



        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));
           // robot.leftMotor.setPower((speed));
            //robot.rightMotor.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());

                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    } */
}
