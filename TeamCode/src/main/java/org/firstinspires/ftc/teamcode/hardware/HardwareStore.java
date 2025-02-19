package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class HardwareStore {
    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    private Arm arm;
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public Servo extensionServo;
    public Intake intake;
    public Servo intakeGrab;
    public Servo intakeRotate;
    public Servo intakeFlip;
    public Grabber grabber;
    public Servo grabberServo;
    public Servo grabberFlip;
    public TouchSensor leftLimitSwitch;
    public TouchSensor rightLimitSwitch;
    public ScoringMechanism scoringMechanism;

    public MecanumDrive drive;

//    public SparkFunOTOSDrive autoDrive;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLimitSwitch = hardwareMap.get(TouchSensor.class, "leftLimitSwitch");
        rightLimitSwitch = hardwareMap.get(TouchSensor.class, "rightLimitSwitch");

        arm = new Arm(liftLeft, liftRight, leftLimitSwitch, rightLimitSwitch);

        intakeGrab = hardwareMap.get(Servo.class, "intakeGrabber");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        extensionServo = hardwareMap.get(Servo.class, "extension");
        intakeFlip = hardwareMap.get(Servo.class, "intakeFlip");

        intake = new Intake(intakeGrab, intakeRotate, intakeFlip, extensionServo);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberFlip = hardwareMap.get(Servo.class, "grabberFlip");

        grabber = new Grabber(grabberServo, grabberFlip);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Arm getArm() {
        return arm;
    }

    public MecanumDrive getDrive(){
        return drive;
    }

    public Intake getIntake(){
        return intake;
    }

    public Grabber getGrabber(){
        return grabber;
    }
}