package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by ian on 8/18/2016.
 */
public class HDriveTeleop extends OpMode {
    public GyroSensor gyro2;
    public ModernRoboticsI2cGyro gyro;
    public double gyroAngle;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    HDriveCalculator calculator;
    public HDriveTeleop(){

    }
    public void init(){
        gyro2 = hardwareMap.gyroSensor.get("gyro");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        calculator = new HDriveCalculator();
        gyro = (ModernRoboticsI2cGyro)gyro2;
        gyro.calibrate();
        while(gyro.isCalibrating()){
            try {
                Thread.sleep(50);
            }
            catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    public void loop(){
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        calculator.calculateMovement(leftX, leftY, rightX, gyro.rawZ());
        leftMotor.setPower(calculator.getLeftDrive());
        rightMotor.setPower(calculator.getRightDrive());
        middleMotor.setPower(calculator.getMiddleDrive());
    }
}
