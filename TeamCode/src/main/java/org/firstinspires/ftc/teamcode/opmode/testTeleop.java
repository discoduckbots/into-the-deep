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

import java.util.ArrayList;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="testTeleop", group= "Linear Opmode")
public class testTeleop extends LinearOpMode {

    Arm arm = null;
    Intake intake = null;
    Grabber grabber = null;
    MecanumDrive drive = null;

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


        waitForStart();


        boolean grabInProgress = false;
        double grabStartTime = 0;
        double grabElapsedTime;

        boolean transferInProgress = false;
        double transferStartTime = 0;
        double transferElapsedTime;




        while (opModeIsActive()) {


            if (gamepad1.dpad_up){
                intake.extend();
            }

            if (gamepad1.dpad_down){
                intake.retract();
            }

            intake.extend(gamepad2.right_stick_y);

            if (gamepad1.y){
                grabber.flipGrabberIn();
            }

            if (gamepad1.left_bumper){
                grabber.onPressFlip();
            }
            else{
                grabber.onReleaseFlip();
            }

            if (gamepad1.right_bumper) {
                grabber.onPressGrabber();
            }
            else {
                grabber.onReleaseGrabber();
            }

            if (gamepad1.a){
                intake.closeIntake();
            }

            if (gamepad1.b){
                intake.openIntake();
            }

            if (gamepad2.dpad_up) {
                arm.lift(1.0);
            }

            else if (gamepad2.dpad_down) {
                arm.lower(1.0);
            }

            else {
                arm.stop();
            }

            if (gamepad2.a) {
                grabber.flipGrabberIn();
            }

            if (gamepad2.b) {
                grabber.flipGrabberOut();
            }
/*
            if (gamepad1.x && !transferInProgress){
                transferStartTime = getRuntime();
                transferInProgress = true;
            }
/*
            if (transferInProgress){
                transferElapsedTime = getRuntime() - transferStartTime;
                transferInProgress = transfer(transferElapsedTime);
            } */

            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("lift_left ", arm.liftLeft.getCurrentPosition());
            telemetry.addData("lift_right ", arm.liftRight.getCurrentPosition());
            telemetry.update();

        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }


}