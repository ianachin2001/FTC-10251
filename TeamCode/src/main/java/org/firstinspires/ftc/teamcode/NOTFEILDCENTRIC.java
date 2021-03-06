package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by ian on 8/18/2016.
 */
@TeleOp(name= "HDrive: Not Field Centric")
public class NOTFEILDCENTRIC extends OpMode {

    public ModernRoboticsI2cGyro gyro;
    public double gyroAngle;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    HDrive2 calculator;
    public NOTFEILDCENTRIC(){

    }
    public void init(){

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");


        calculator = new HDrive2();
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = -gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        calculator.calculateMovement(leftX, leftY, rightX, 0);
        leftMotor.setPower(calculator.getLeftDrive());
        rightMotor.setPower(calculator.getRightDrive());
        middleMotor.setPower(calculator.getMiddleDrive());

    }
}
