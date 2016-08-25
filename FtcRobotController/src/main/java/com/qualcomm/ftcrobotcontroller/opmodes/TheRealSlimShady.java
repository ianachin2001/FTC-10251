package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by ldar on 11/29/2015.
 */
public class TheRealSlimShady extends OpMode {

    final static double ARM_MIN_RANGE  = 0;
    final static double ARM_MAX_RANGE  = 1.0;


    DcMotor motorBase1;
    DcMotor motorBase2;
    //  DcMotor motorIntake1;
    //  DcMotor motorIntake2;
    DcMotor Hook1;
    DcMotor Hook2;
    Servo HookServ;
    Servo ClimbServ;
    Servo HangLeft;
    Servo HangRight;

    double HookPosition;
    double ClimbPosition;

    public TheRealSlimShady(){

    }

    @Override
    public void init() {
        motorBase1 = hardwareMap.dcMotor.get("Right_Motor");
        motorBase2 = hardwareMap.dcMotor.get("Left_Motor");
        motorBase2.setDirection(DcMotor.Direction.REVERSE);
        Hook1 = hardwareMap.dcMotor.get("Hook1");
        Hook2 = hardwareMap.dcMotor.get("Hook2");
        // motorIntake1 = hardwareMap.dcMotor.get("Intake1");
        // motorIntake2 = hardwareMap.dcMotor.get("Intake2");
        HookServ = hardwareMap.servo.get("HookServ");
        //ClimbServ = hardwareMap.servo.get("ClimbServ");
        HangLeft = hardwareMap.servo.get("hl");
        HangRight = hardwareMap.servo.get("hr");

        gamepad2.left_trigger = 0;
        gamepad2.right_trigger = 0;
        gamepad1.left_trigger = 0;
        gamepad1.right_trigger = 0;
        ClimbServ.setPosition(.5);




        HookPosition = 0;
        ClimbPosition = 0;

    }

    @Override
    public void loop() {

        //Drive Base
        float throttle = -gamepad1.left_stick_y;
        float throttle2 = gamepad1.right_stick_y;

        throttle = Range.clip(throttle, -1, 1);
        throttle2 = Range.clip(throttle2, -1, 1);

        throttle = (float)scaleInput(throttle);
        throttle2 = (float)scaleInput(throttle2);

        motorBase1.setPower(throttle);
        motorBase2.setPower(-throttle2);


//extra code for driving with one stick
        //float throttle = -gamepad1.left_stick_y;
        //float direction = gamepad1.left_stick_x;
        //float right = throttle - direction;
        //float left = throttle + direction;
     /*right = Range.clip(right, -1, 1);
     left = Range.clip(left, -1, 1);
     right = (float)scaleInput(right);
     left = (float)scaleInput(left);
     motorBase1.setPower(right);
     motorBase2.setPower(left);*/

        //Hook in and out
        double Hook =  -gamepad2.left_stick_y;

        Hook = scaleInput(Hook);
        Hook = Range.clip(Hook, -1, 1);

        Hook1.setPower(Hook);
        Hook2.setPower(Hook);

        //Hook Position
        double servoPower = -gamepad2.right_stick_y;
        if(servoPower >= -.01 && servoPower <= .01){
            servoPower =(float) .5; //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        } else{
            servoPower = scaleServ((float)servoPower);
        }
        servoPower = Range.clip(servoPower, 0, 1);
        HookServ.setPosition(servoPower);

        /*float servoLeft = gamepad2.left_trigger;
        float servoRight = gamepad2.right_trigger;
        float servoLeft1 = gamepad1.left_trigger;
        float servoRight1 = gamepad1.right_trigger;

        if(servoRight == .5){
            servoLeft =0;
        }
        if(servoRight1 == .5){
            servoLeft = 0;
        }

        /*if(servoLeft != 0){
            ClimbServ.setPosition(.25);
        } else if(servoRight != 0){
            ClimbServ.setPosition(.75);
        }else if(servoLeft1 != 0){
            ClimbServ.setPosition(.25);
        } else if(servoRight1 != 0){
            ClimbServ.setPosition(.75);
        }else{
            ClimbServ.setPosition(.5);
        }*/


        AnalogInput potentiometer = hardwareMap.analogInput.get("hpotter");
        int value1 = potentiometer.getValue();
        telemetry.addData ("1", "potentiometer " + value1);


        //I added this so you can move the hook to different positions, they are test positions
        if(gamepad2.a){



            if(value1 > 520){
                HookServ.setPosition(.65);
            }else if(value1 > 491){
                HookServ.setPosition(.55);
            }else if(value1 < 460){
                HookServ.setPosition(.35);
            }else if(value1 < 489){
                HookServ.setPosition(.45);
            }else{
                HookServ.setPosition(.5);
            }

        }
        if(gamepad2.b){
            if(value1 > 590){
                HookServ.setPosition(.65);
            }else if(value1 > 561){
                HookServ.setPosition(.55);
            }else if(value1 < 530){
                HookServ.setPosition(.35);
            }else if(value1 < 559){
                HookServ.setPosition(.45);
            }else{
                HookServ.setPosition(.5);
            }
        }
        if(gamepad1.y){
            HangLeft.setPosition(.25);
        }else if(gamepad1.x){
            HangLeft.setPosition(.75);
        }else{
            HangLeft.setPosition(.5);
        }
        if(gamepad1.a){
            HangRight.setPosition(.25);
        }else if(gamepad1.b){
            HangRight.setPosition(.75);
        }else{
            HangRight.setPosition(.5);
        }
        if(gamepad2.x){
            if(Hook2.getPower() != 0){
                HookServ.setPosition(0);
            }else{
                HookServ.setPosition(1);
            }
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
    public static float scaleServ(float pow){

        float power =0;
        if(pow > 0) {
            power = (pow * pow + 1) / 2;
        }else {
            power = (-pow * pow + 1)/ 2;
        }

        return power;
    }

    //this method uses the potempitonometer to move the hookserv to different positions
    public static double servTo(int value1, int location){
        double value = -value1;
        value = location-value;
        value = value/300;
        if(value > 0){
            value = (value + 1)/2;
        }else if(value < 0){
            value = (value + 1)/2;
        }

        //so it doesn't go too slow
        if(value < .58 && value > 5){
            value = .58;
        }else if(value > .42 && value < 5){
            value = .42;
        }
        return value;
    }
}
