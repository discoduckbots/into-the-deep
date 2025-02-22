package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.ScoringMechanism;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Fancy Cancelable Teleop", group= "Competition Opmodes")
public class FancyCancelableTeleop extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    Arm arm = null;
    Intake intake = null;
    Grabber grabber = null;
    TouchSensor leftLimitSwitch = null;
    TouchSensor rightLimitSwitch = null;
    MecanumDrive drive = null;

    private double THROTTLE = .595;
    private double TURN_THROTTLE = 0.375;
    private double LIFT_SPEED = 0.9;
    private double LOWER_SPEED = 0.5;
    private double EXTENSION_SPEED = 0.7;
    private boolean inGrabPosition = false;
    boolean minute = false;
    boolean endgame = false;
    boolean tenSec = false;
    boolean rumbling = false;
    boolean transferInProgress = false;
    double transferStartTime = 0;
    double transferElapsedTime;
    private ElapsedTime runtime = new ElapsedTime();
    private Pose2d lastPositionA = null;
    private Pose2d lastPositionB = null;
    boolean lastPositionPressed = false;
    Action moveToLastPosAction = null;
    String transferState = "0";

    /**
     * Transfers a sample from the intake to outtake using state and elapsed time instead of sleeping
     *
     * Preconditions: You have a sample grabbed in the intake and the intake is at center position
     * Postconditions: The robot is holding the sample in the outtake flipped backwards
     *
     * @// TODO: 2/17/2025 - Need to consider what happens if the outtake/grabber is flipped out when we start
     * @// TODO: 2/17/2025 - Need to consider if the intake is (partially) extended when we start
     *
     * @param elapsedTime - how long the transfer process has taken (seconds)
     * @return boolean - whether transfer is still in progress
     */
    public boolean transfer(double elapsedTime){

        String TS_0_NOT_STARTED = "0";

        String TS_1_1_OPEN_OUTTAKE = "1_1";

        String TS_2_1_INTAKE_UP = "2_1";
        String TS_2_2_OUTTAKE_GRAB = "2_2";
        String TS_2_3_INTAKE_OPEN = "2_3";

        String TS_3_1_OUTTAKE_FLIP = "3_1";

        if (TS_2_3_INTAKE_OPEN.equals(transferState)){
            grabber.flipGrabberOut();
            transferState = TS_3_1_OUTTAKE_FLIP;
        }

        if(TS_2_2_OUTTAKE_GRAB.equals(transferState) && elapsedTime > 0.5){
            intake.openIntake();
            transferState = TS_2_3_INTAKE_OPEN;
        }

        if (TS_2_1_INTAKE_UP.equals(transferState) && elapsedTime > 0.3){
            grabber.closeGrabber();
            transferState = TS_2_2_OUTTAKE_GRAB;
        }

        if (TS_1_1_OPEN_OUTTAKE.equals(transferState)){
            intake.flipIntakeUp();
            transferState = TS_2_1_INTAKE_UP;
        }

        if (TS_0_NOT_STARTED.equals(transferState)){
            grabber.openGrabber();
            transferState = TS_1_1_OPEN_OUTTAKE;
        }

        if(TS_3_1_OUTTAKE_FLIP.equalsIgnoreCase(transferState)){
            transferState = TS_0_NOT_STARTED;
            return false;
        }

        return true;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        int extendPosition = 0;

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        intake = hardwareStore.getIntake();
        grabber = hardwareStore.getGrabber();
        drive = hardwareStore.getDrive();
        leftLimitSwitch = hardwareStore.leftLimitSwitch;
        rightLimitSwitch = hardwareStore.rightLimitSwitch;

        waitForStart();

        intake.rotateIntakeTo0();

        while (opModeIsActive()) {
            /* Gamepad 1 Stuff */
            driveControl(drive);

            if(gamepad1.dpad_up) {
                intake.extend();
            } else {
                intake.retract();
            }
            if(gamepad1.a) {
                intake.openIntake();
                intake.flipIntakeWonky();
            }

            if (gamepad1.right_bumper) {
                intake.onPressIntake();
            }
            else {
                intake.onReleaseIntake();
            }

            if (gamepad1.y){
                intake.flipIntakeDown();
                sleep(250);
                intake.closeIntake();
                sleep(250);
                intake.flipIntakeUp();
            }
            if (gamepad1.b) {
                intake.onPressRotate();
            }
            else {
                intake.onReleaseRotate();
            }




            /* Gamepad 2 Stuff */

            //intake.extend(gamepad2.right_stick_y);

            /* Arm Controls - DPAD */
            if (gamepad2.dpad_up){
                arm.lift(LIFT_SPEED);
            }
            else if (gamepad2.dpad_down){
                arm.lower(LOWER_SPEED);
            }
            else if (gamepad2.dpad_right) {
                arm.liftByEncoder(Arm.LIFT_PLACE_SPECIMEN, LIFT_SPEED);
            }
            else{
                arm.stop();
            }

            /* Intake Controls - LEFT TRIGGER/BUMPER */
            if (gamepad2.left_trigger > 0.2) {
                intake.onPressFlip();
            }
            else {
                intake.onReleaseFlip();
            }

            /*if (gamepad2.left_bumper) {
                intake.onPressIntake();
            }
            else {
                intake.onReleaseIntake();
            } */

            /* Grabber/Outtake Controls - RIGHT TRIGGER/BUMPER */
            if (gamepad2.right_trigger > 0.2) {
                grabber.onPressFlip();
            }
            else {
                grabber.onReleaseFlip();
            }

            if (gamepad2.right_bumper) {
                grabber.onPressGrabber();
            }
            else {
                grabber.onReleaseGrabber();
            }

            if (gamepad2.a) {
                intake.openIntake();
                intake.flipIntakeWonky();
            }

            /*if (gamepad2.b) {
                intake.onPressRotate();
            }
            else {
                intake.onReleaseRotate();
            } */

            if (gamepad2.dpad_left) {
                intake.flipIntakeDown();
            }

            if (gamepad2.x && !transferInProgress){
                transferStartTime = getRuntime();
                transferInProgress = true;
            }

            if (transferInProgress){
                transferElapsedTime = getRuntime() - transferStartTime;
                transferInProgress = transfer(transferElapsedTime);
            }

            if (gamepad2.y){
                intake.flipIntakeDown();
                sleep(250);
                intake.closeIntake();
                sleep(250);
                intake.flipIntakeUp();
            }

            if (runtime.time() >= 60 && !minute) {
                gamepad2.rumbleBlips(2);
                minute = true;
            }
            if (runtime.time() >= 90 && !endgame) {
                gamepad2.rumbleBlips(3);
                endgame = true;
            }
            if (runtime.time() >= 110 && !tenSec) {
                gamepad2.rumbleBlips(5);
                tenSec = true;
            }




            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("lift_left ", arm.liftLeft.getCurrentPosition());
            telemetry.addData("lift_right ", arm.liftRight.getCurrentPosition());
            telemetry.addData("l_limit_switch ", leftLimitSwitch.getValue());
            telemetry.addData("r_limit_switch ", rightLimitSwitch.getValue());
            telemetry.update();

        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }

    private void driveControl(MecanumDrive drive) {
        Pose2d poseEstimate = drive.localizer.getPose();
        drive.updatePoseEstimate();
        Log.d("LOC", "x = " + poseEstimate.position.x +
                " y= " + poseEstimate.position.y +
                " heading " + Math.toDegrees(poseEstimate.heading.real));
        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * THROTTLE,
                                -gamepad1.left_stick_x * THROTTLE
                        ),
                        -gamepad1.right_stick_x * TURN_THROTTLE
                ));

                drive.updatePoseEstimate();
                if (gamepad2.left_bumper) {
                    lastPositionA = poseEstimate;
                    Log.d("LAST", "Setting last position to " + lastPositionA);
                }
                if (gamepad2.dpad_right) {
                    lastPositionB = poseEstimate;
                    Log.d("LAST", "Setting last position to " + lastPositionB);
                }
                if (gamepad1.left_bumper || gamepad1.dpad_right) {
                    if (!lastPositionPressed) {
                        lastPositionPressed = true;
                        Pose2d lastPosition = null;
                        if (gamepad1.dpad_left) lastPosition = lastPositionA;
                        else lastPosition = lastPositionB;
                        if (lastPosition != null) {
                            currentMode = Mode.AUTOMATIC_CONTROL;
                            TrajectoryActionBuilder moveToLastPosition = drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(new Vector2d(lastPosition.position.x, lastPosition.position.y), lastPosition.heading);
                            moveToLastPosAction = moveToLastPosition.build();

                        }
                    }
                }
                else {
                    lastPositionPressed = false;
                }

                break;
            case AUTOMATIC_CONTROL:
                if (gamepad1.a) {
                    //drive.
                    currentMode = Mode.DRIVER_CONTROL;
                    return;
                }
               if (moveToLastPosAction != null) {
                   TelemetryPacket packet = new TelemetryPacket();
                   moveToLastPosAction.preview(packet.fieldOverlay());
                   if (!moveToLastPosAction.run(packet)) {
                       currentMode = Mode.DRIVER_CONTROL;
                   }

               }
        }
    }

}