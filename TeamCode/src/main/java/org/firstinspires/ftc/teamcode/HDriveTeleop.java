package org.firstinspires.ftc.teamcode;
<<<<<<< HEAD
import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
=======
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
<<<<<<< HEAD
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
=======
import com.qualcomm.robotcore.hardware.I2cDevice;
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d

/**
 * Created by HIRSH on 8/18/2016.
 */
@TeleOp(name= "HDriveTeleop")
public class HDriveTeleop extends OpMode {
<<<<<<< HEAD
    //double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    Orientation angles;
    BNO055IMU imu;
    String angleDouble = "hi";
=======
<<<<<<< HEAD
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    //public GyroSensor gyro2;
    //public ModernRoboticsI2cGyro gyro;
=======
    public GyroSensor gyro2;
    public ModernRoboticsI2cGyro gyro;
>>>>>>> origin/master
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
    public double gyroAngle;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
<<<<<<< HEAD
    HDriveFCCalc calculator;
    CRServo servo;

    //AdafruitIMU boschBNO055;
=======
    HDrive2 calculator;
<<<<<<< HEAD
    I2cDevice imu;
    AdafruitGyro adafruitImu;
=======
>>>>>>> origin/master
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
    public HDriveTeleop(){

    }
    public void init(){
<<<<<<< HEAD
       /*try {
           boschBNO055 = new AdafruitIMU(hardwareMap, "bno055"
=======
<<<<<<< HEAD
        /*try {
            boschBNO055 = new AdafruitIMU(hardwareMap, "bno055"
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d

                   //The following was required when the definition of the "I2cDevice" class was incomplete.
                   //, "cdim", 5

<<<<<<< HEAD
                   , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                   //addressing
                   , (byte)AdafruitIMU.OPERATION_MODE_IMU);
       } catch (RobotCoreException e){
           Log.i("FtcRobotController", "Exception: " + e.getMessage());
       }*/
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
        //servo = hardwareMap.crservo.get("servo");
        calculator = new HDriveFCCalc();
=======
                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }*/
        //gyro2 = hardwareMap.gyroSensor.get("gyro");
        imu = hardwareMap.i2cDevice.get("imu");
        adafruitImu = new AdafruitGyro(imu);
=======
        gyro2 = hardwareMap.gyroSensor.get("gyro");
>>>>>>> origin/master
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor"); //luke is better

        calculator = new HDrive2();
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
<<<<<<< HEAD
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
<<<<<<< HEAD

        //  runOpMode();

        //waitForStart();
        //CRServo servo = this.hardwareMap.crservo.get("servo")
=======
        /*gyro = (ModernRoboticsI2cGyro)gyro2;
=======
        gyro = (ModernRoboticsI2cGyro)gyro2;
>>>>>>> origin/master
        gyro.calibrate();
        while(gyro.isCalibrating()){
            try {
                Thread.sleep(50);
            }
            catch(InterruptedException e){
                e.printStackTrace();
            }
        }*/
>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d
    }

    public void loop(){
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
<<<<<<< HEAD
        calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble));
        leftMotor.setPower(.7*calculator.getLeftDrive());
        rightMotor.setPower(.7*calculator.getRightDrive());
        middleMotor.setPower(calculator.getMiddleDrive());
       /*boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
     /*
      * Send whatever telemetry data you want back to driver station.
      */
        //telemetry.addData("Text", "*** Robot Data***");
       /*telemetry.addData("Headings(yaw): ",
               String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
       telemetry.addData("Pitches: ",
               String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
       telemetry.addData("Max I2C read interval: ",
               String.format("%4.4f ms. Average interval: %4.4f ms.", boschBNO055.maxReadInterval
                       , boschBNO055.avgReadInterval));*/
=======
        calculator.calculateMovement(leftX, leftY, rightX, 0);
        leftMotor.setPower(calculator.getLeftDrive());
        rightMotor.setPower(calculator.getRightDrive());
        middleMotor.setPower(calculator.getMiddleDrive());
<<<<<<< HEAD
        double[] gyroResults = adafruitImu.getAngles();
        telemetry.addData("Text", "Gyro yaw: " + gyroResults[0] + " " + gyroResults[1] + " " + gyroResults[2]);

>>>>>>> 24c8a2ff022ce31916266d97ec9b1a7a72fceb6d

=======
>>>>>>> origin/master
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

