package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by ian on 8/18/2016.
 */
public class HDriveCalculator {
    double driveLeft;
    double driveRight;
    double driveMiddle;
    public HDriveCalculator(){

    }
    public void calculateMovement(double joystickX, double joystickY, double turnStickX, double angle){
        double scaleFactor = Math.sqrt(joystickX * joystickX + joystickY * joystickY);
        double moveAngle = Math.atan(joystickY/joystickX);
        double robotAngle = angle;
        double xMove = scaleFactor * (Math.cos(moveAngle - robotAngle));
        double yMove = scaleFactor * Math.sin(moveAngle - robotAngle);
        double turn = turnStickX/2;
        driveLeft = yMove + turn;
        driveRight = yMove - turn;
        driveMiddle = xMove;
        if(driveLeft > 1){
            driveRight = driveRight * (1/driveLeft);
            driveMiddle = driveMiddle * (1/driveLeft);
            driveLeft = 1;
        }
        else if(driveRight > 1){
            driveLeft = driveLeft * (1/driveRight);
            driveMiddle = driveMiddle * (1/driveRight);
            driveRight = 1;
        }
    }
    public double getLeftDrive(){
        return driveLeft;
    }
    public double getRightDrive(){
        return driveRight;
    }
    public double getMiddleDrive(){
        return driveMiddle;
    }
}
