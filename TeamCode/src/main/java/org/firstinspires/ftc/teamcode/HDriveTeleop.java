package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * Created by HIRSH on 8/18/2016.
 */
@TeleOp(name= "HDriveTeleop")
public class HDriveTeleop extends OpMode {
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    //public GyroSensor gyro2;
    //public ModernRoboticsI2cGyro gyro;
    public double gyroAngle;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    HDrive2 calculator;
    I2cDevice imu;
    AdafruitGyro adafruitImu;
    public HDriveTeleop(){

    }
    public void init(){
        /*try {
            boschBNO055 = new AdafruitIMU(hardwareMap, "bno055"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }*/
        //gyro2 = hardwareMap.gyroSensor.get("gyro");
        imu = hardwareMap.i2cDevice.get("imu");
        adafruitImu = new AdafruitGyro(imu);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor"); //luke is better

        calculator = new HDrive2();
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        /*gyro = (ModernRoboticsI2cGyro)gyro2;
        gyro.calibrate();
        while(gyro.isCalibrating()){
            try {
                Thread.sleep(50);
            }
            catch(InterruptedException e){
                e.printStackTrace();
            }
        }*/
    }

    public void loop(){
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        calculator.calculateMovement(leftX, leftY, rightX, 0);
        leftMotor.setPower(calculator.getLeftDrive());
        rightMotor.setPower(calculator.getRightDrive());
        middleMotor.setPower(calculator.getMiddleDrive());
        double[] gyroResults = adafruitImu.getAngles();
        telemetry.addData("Text", "Gyro yaw: " + gyroResults[0] + " " + gyroResults[1] + " " + gyroResults[2]);


    }
}
