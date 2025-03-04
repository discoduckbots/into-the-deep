package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public TouchSensor leftLimitSwitch;
    public TouchSensor rightLimitSwitch;

    public static final int LIFT_BASKET = 1100;

    public static final int LIFT_BASKET_LOWER = 1000;
    public static final int LIFT_PLACE_PRELOAD_SPECIMEN = 1980;
    public static final int LIFT_PLACE_SPECIMEN = 237;
    public static final int LIFT_GRAB_FROM_WALL = 0;
    public static final int LIFT_ABOVE_BAR = 630;
    public static final int LIFT_EXTRA_ABOVE = 680;
    public static final int LIFT_BELOW_BAR = 400;

    // high goal - L: -4418, R: -6302
// above bar - L: 2134, R:4154
    // below bar - L-1193, R: -3206

    public Arm(DcMotorEx liftMotor1, DcMotorEx liftMotor2, TouchSensor leftLimitSwitch, TouchSensor rightLimitSwitch) {

        this.liftLeft = liftMotor1;
        this.liftLeft.setDirection(DcMotor.Direction.REVERSE);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.liftRight = liftMotor2;
        this.liftRight.setDirection(DcMotor.Direction.FORWARD);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
    }

    public int getLiftPos() {
        return liftLeft.getCurrentPosition();
    }

    public int getLiftPos2() {
        return liftRight.getCurrentPosition();
    }

    public void lift(double power) {
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(power);
        liftRight.setPower(power);
        Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());

    }

    public void lower(double power) {
        if (!leftLimitSwitch.isPressed() && !rightLimitSwitch.isPressed()) {
            liftLeft.setDirection(DcMotorEx.Direction.FORWARD);
            liftRight.setDirection(DcMotorEx.Direction.REVERSE);
            liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(power);
            liftRight.setPower(power);
            Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
            liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stop() {
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void liftByEncoder(int position, double power) {
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft.setTargetPosition(position);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(position);
        liftRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftLeft.setVelocity(1200);
        liftRight.setVelocity(1200);

    }

    public void lowerByTouch(double power) {
        stop();
        while (!leftLimitSwitch.isPressed()) {
            lower(power);
        }
    }

}
