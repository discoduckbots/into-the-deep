package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private static final double INTAKE_OPEN_POS= 0.8;
    private static final double INTAKE_CLOSE_POS = 0.4;
    private static final double INTAKE_UP_POS = 0.25;
    private static final double INTAKE_DOWN_POS = 0.77;
    private static final double INTAKE_DOWN_UP = .72;
    private static final double INTAKE_DOWN_LOWER = 0.8;
    private static final double INTAKE_ROTATE_90_POS = 1.0; //b
    private static final double INTAKE_ROTATE_0_POS = 0.0; //a
    public static final double EXTENSION_SERVO_OUT_POS = 0;
    public static final double EXTENSION_SERVO_IN_POS = 0;

    private boolean isRotatedTo0 = true;
    private boolean isIntakeOpen;
    private boolean isIntakeUp = true;
    private boolean buttonPressIntake = false;
    private boolean buttonPressFlip = false;
    private boolean buttonPressRotate = false;
    public Servo intakeGrab;
    public Servo intakeRotate;
    public Servo intakeFlip;
    public Servo extensionServo;


    public Intake(Servo intakeGrab, Servo intakeRotate, Servo intakeFlip, Servo extensionServo) {
        this.intakeGrab = intakeGrab;
        this.intakeRotate = intakeRotate;
        this.intakeFlip = intakeFlip;
        this.extensionServo = extensionServo;
    }

    public void openIntake() {
        intakeGrab.setPosition(INTAKE_OPEN_POS);
        isIntakeOpen = true;
    }

    public void closeIntake() {
        intakeGrab.setPosition(INTAKE_CLOSE_POS);
        isIntakeOpen = false;
    }

    public void onPressIntake() {

        if (buttonPressIntake) return;
        buttonPressIntake = true;
        if (isIntakeOpen) {
            isIntakeOpen = false;
            closeIntake();
        }
        else {
            isIntakeOpen = true;
            openIntake();
        }
    }

    public void onReleaseIntake() {
        buttonPressIntake = false;
    }

    public void flipIntakeUp() {
        if (!isRotatedTo0) {
            rotateIntakeTo0();
        }
        if (isRotatedTo0) {
            intakeFlip.setPosition(INTAKE_UP_POS);
            isIntakeUp = true;
        }
    }

    public void flipIntakeDown() {
        intakeGrab.close();
        intakeFlip.setPosition(INTAKE_DOWN_POS);
        isIntakeUp = false;
    }

    public void flipIntakeDownUp() {
        intakeFlip.setPosition(INTAKE_DOWN_UP);
        isIntakeUp = false;
    }

    public void onPressFlip() {

        if (buttonPressFlip) return;
        buttonPressFlip = true;
        if (isIntakeUp) {
            isIntakeUp = false;
            flipIntakeDownUp();
        }
        else {
            isIntakeUp = true;
            flipIntakeUp();
        }
    }

    public void onReleaseFlip() {
        buttonPressFlip = false;
    }

    public void rotateIntakeTo90() {
        intakeRotate.setPosition(INTAKE_ROTATE_90_POS);
        isRotatedTo0 = false;
    }

    public void rotateIntakeTo0() {
        intakeRotate.setPosition(INTAKE_ROTATE_0_POS);
        isRotatedTo0 = true;
    }
    public void onPressRotate() {

        if (buttonPressRotate) return;
        buttonPressRotate = true;
        if (isRotatedTo0) {
            isRotatedTo0 = false;
            rotateIntakeTo90();
        }
        else {
            isRotatedTo0 = true;
            rotateIntakeTo0();
        }
    }

    public void onReleaseRotate() {
        buttonPressRotate = false;
    }


    public void rotateIntake(double position) {
        intakeRotate.setPosition(position);
    }

    public void extend(){
        extensionServo.setPosition(EXTENSION_SERVO_OUT_POS);
    }

    public void retract(){
        extensionServo.setPosition(EXTENSION_SERVO_IN_POS);
    }
}
