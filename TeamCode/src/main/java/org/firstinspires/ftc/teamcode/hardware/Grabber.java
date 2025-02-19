package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    Servo grabberServo = null;
    Servo grabberFlip = null;

    private static final double GRABBER_OPEN_POS = 0.65;
    private static final double GRABBER_CLOSE_POS = 0;
    private static final double AUTO_GRAB_POS = 0.97;
    public static final double GRABBER_IN_POS = 0.94;
    private static final double GRABBER_IN_AUTO = 0.02;
    public static final double GRABBER_OUT_POS = 0.16;
    public static final double GRABBER_MID_POS = 0.35;
    public double[] grabberFlipPos = {GRABBER_IN_POS, GRABBER_MID_POS, GRABBER_OUT_POS};

    private boolean grabberIn;
    private boolean grabberOut;
    private boolean grabberMid;
    private boolean grabberOpen = false;
    private boolean buttonPressGrabber = false;
    private boolean buttonPressFlip = false;


    public Grabber(Servo grabberServo, Servo grabberFlip) {

        this.grabberServo = grabberServo;
        this.grabberFlip = grabberFlip;
    }

    public void closeGrabber() {
        grabberServo.setPosition(GRABBER_CLOSE_POS);
        grabberOpen = false;
    }

    public void autoCloseGrabber() {
        grabberServo.setPosition(AUTO_GRAB_POS);
    }

    public void openGrabber() {
        grabberServo.setPosition(GRABBER_OPEN_POS);
        grabberOpen = true;
    }

    public void onPressGrabber() {

        if (buttonPressGrabber) return;
        buttonPressGrabber = true;
        if (grabberOpen) {
            grabberOpen = false;
            closeGrabber();
        }
        else {
            grabberOpen = true;
            openGrabber();
        }
    }

    public void onReleaseGrabber() {
        buttonPressGrabber = false;
    }

    public boolean isGrabberOpen() {
        return grabberOpen;
    }

    public void flipGrabberOut() {
        closeGrabber();

        if (!grabberOpen) {
            grabberFlip.setDirection(Servo.Direction.FORWARD);
            //only if linear slide is all the way down
            grabberFlip.setPosition(GRABBER_OUT_POS);
            grabberOut = true;
            grabberIn = false;
            grabberMid = false;
        }
    }

    public void flipGrabberIn() {
        closeGrabber();

        if (!grabberOpen) {
            grabberFlip.setDirection(Servo.Direction.FORWARD); //test comment
            grabberFlip.setPosition(GRABBER_IN_POS);
            grabberIn = true;
            grabberOut = false;
            grabberMid = false;

        }
    }

    public void flipGrabberInAuto() {
        if (grabberOpen) {
            closeGrabber();
        }
        if (!grabberOpen) {
            grabberFlip.setDirection(Servo.Direction.FORWARD); //test comment
            grabberFlip.setPosition(GRABBER_IN_AUTO);
            grabberIn = true;
            grabberOut = false;
            grabberMid = false;

        }
    }


    public void flipGrabberMiddle() {
        grabberFlip.setPosition(GRABBER_MID_POS);
        grabberMid = true;
        grabberOut = false;
        grabberIn = false;
    }

    public void onPressFlip() {

        if (buttonPressFlip) return;
        buttonPressFlip = true;
        if (grabberIn) {
            grabberIn = false;
            flipGrabberOut();
        }
        else {
            grabberIn = true;
            flipGrabberIn();
        }
    }

    public void onReleaseFlip() {
        buttonPressFlip = false;
    }

    /*
    newOnPressFlip() {
        if (buttonPressFlip) {
            return;
        }
        grabberFlip.setPosition(grabberFlipPos[])
    } */

    public boolean isGrabberIn() {
        return grabberIn;
    }

    public boolean isGrabberOut() {
        return grabberOut;
    }

}