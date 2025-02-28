package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

@Config
@Autonomous(name = "SpecimenAuto", group = "Autonomous")
public class SpecimenAuto extends DuckbotAuto {

    MecanumDrive drive = null;
    Grabber teleGrabber = null;
    Intake teleIntake = null;

    private static final TranslationalVelConstraint SLOW_VEL= new TranslationalVelConstraint(50);
    private static final ProfileAccelConstraint SLOW_ACC = new ProfileAccelConstraint(-30, 30);
    private static final TranslationalVelConstraint MED_VEL= new TranslationalVelConstraint(75);
    private static final ProfileAccelConstraint MED_ACC = new ProfileAccelConstraint(-30, 50);
    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoArm arm = new AutoArm(hardwareStore);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();
        teleGrabber = hardwareStore.getGrabber();
        teleIntake = hardwareStore.getIntake();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        teleIntake.rotateIntakeTo0();
        teleGrabber.closeGrabber();



        TrajectoryActionBuilder scorePreload = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(34, 17)); // move to bar
        /*
        TrajectoryActionBuilder pushFirstSample = scorePreload.endTrajectory().fresh()
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(31, -25.5), Math.toRadians(0)) // x was 30
                //.splineToConstantHeading(new Vector2d(22, -18), Math.toRadians(0)) //back up from bar
                .splineToConstantHeading(new Vector2d(56, -34), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to first sample
                .splineToConstantHeading(new Vector2d(18, -34), Math.toRadians(0), SLOW_VEL, SLOW_ACC); //first move to wall

        TrajectoryActionBuilder pushSecondSample = pushFirstSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(46, -39.5), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to second sample
                .splineToConstantHeading(new Vector2d(15, -39), Math.toRadians(0), SLOW_VEL, SLOW_ACC); //second move to wall

        TrajectoryActionBuilder pushThirdSample = pushSecondSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(48, -49), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to third sample
                .splineToConstantHeading(new Vector2d(8, -48.5), Math.toRadians(-90)); //third move to wall */

        TrajectoryActionBuilder transferFirstSample = scorePreload.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(14.5, -42, 0), Math.toRadians(-90), MED_VEL, MED_ACC);

        TrajectoryActionBuilder transferSecondSample = transferFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(14.5, -50), Math.toRadians(0), MED_VEL, MED_ACC);

        /*TrajectoryActionBuilder pushThirdSample = transferSecondSample.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48, -52), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to third sample
                .splineToConstantHeading(new Vector2d(8, -52), Math.toRadians(-90)); //third move to wall */

        TrajectoryActionBuilder grabFirstSpecimen = transferSecondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, -28, Math.toRadians(0)), Math.toRadians(90));

        TrajectoryActionBuilder scoreFirstSpecimen = drive.actionBuilder(new Pose2d(0,0, 0))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30,41), Math.toRadians(0))
                .strafeTo(new Vector2d(35.5, 41));

        TrajectoryActionBuilder grabSecondSpecimen = scoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, 41), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-2, 0, Math.toRadians(0)), Math.toRadians(90));

        TrajectoryActionBuilder scoreSecondSpecimen = drive.actionBuilder(new Pose2d(0,0, 0))
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(22,44), Math.toRadians(0))
                .strafeTo(new Vector2d(35.5, 44));

        TrajectoryActionBuilder grabThirdSpecimen = scoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(30, 44), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-1, 0), Math.toRadians(0));

        TrajectoryActionBuilder scoreThirdSpecimen = drive.actionBuilder(new Pose2d(0,0, 0))
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(24, 39), Math.toRadians(0))
                .strafeTo(new Vector2d(35.5, 39));

        TrajectoryActionBuilder parkInObservation = scoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(30, 39), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0));


        telemetry.addData("ready to start", "");
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                grabber.grabberIn(),
                grabber.grabberGrab(),
                intake.retract()
                )
        );

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;

            // Drive to preload
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scorePreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN)
                            ),
                            grabber.grabberMiddle(),
                            new SleepAction(0.3),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            //transfer first sample
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberIn(),
                            new ParallelAction(
                                    transferFirstSample.build(),
                                    arm.liftToTargetPosition(0)
                            ),
                            intake.extend(0.93),
                            grabber.grabberRelease(),
                            intake.intakeDown(),
                            new SleepAction(0.6),
                            intake.intakeClose(),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    intake.intakeUp(),
                                    intake.retract()
                            ),
                            new SleepAction(0.5),
                            grabber.grabberGrab(),
                            new SleepAction(0.2),
                            intake.intakeOpen(),
                            grabber.grabberOut(),
                            new SleepAction(0.8),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            //transfer second sample
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    grabber.grabberIn(),
                                    transferSecondSample.build(),
                                    intake.extend(0.93)
                                    ),
                            intake.intakeDown(),
                            grabber.grabberRelease(),
                            new SleepAction(0.5),
                            intake.intakeClose(),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    intake.intakeUp(),
                                    intake.retract()
                            ),
                            new SleepAction(0.5),
                            grabber.grabberGrab(),
                            new SleepAction(0.2),
                            intake.intakeOpen(),
                            grabber.grabberOut(),
                            new SleepAction(0.8),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );
/*
            // push third sample
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.retract(),
                                    grabber.grabberRelease(),
                                    pushThirdSample.build()
                            )
                    )
            ); /*
/*
            // push samples
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.retract(),
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                    pushFirstSample.build()
                            ),
                            new ParallelAction(
                                    grabber.grabberOut(),
                                    pushSecondSample.build()
                            ),
                            new SleepAction(.2),
                            new ParallelAction(
                                    intake.retract(),
                                    grabber.grabberRelease(),
                                    pushThirdSample.build()
                            )
                    )
            ); */

            // grab first specimen
            Actions.runBlocking(
                    new SequentialAction(
                            grabFirstSpecimen.build(),
                            grabber.grabberGrab(),
                            new SleepAction(0.3)
                    )
            );
            drive.localizer.setPose(new Pose2d(0, 0, 0));

            //score first specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scoreFirstSpecimen.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN -20),
                                    grabber.grabberIn()
                            ),
                            grabber.grabberMiddle(),
                            new SleepAction(0.3),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            //grab second specimen
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberOut(),
                            new ParallelAction(
                                    grabSecondSpecimen.build(),
                                    arm.liftToTargetPosition(0),
                                    grabber.grabberRelease()
                            ),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.3)
                    )
            );

            drive.localizer.setPose(new Pose2d(0, 0, 0));

            //score second specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scoreSecondSpecimen.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN -20),
                                    grabber.grabberIn()
                            ),
                            grabber.grabberMiddle(),
                            new SleepAction(0.3),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            //grab third specimen
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberOut(),
                            new ParallelAction(
                                    grabThirdSpecimen.build(),
                                    arm.liftToTargetPosition(0),
                                    grabber.grabberRelease()
                            ),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.3)
                    )
            );

            drive.localizer.setPose(new Pose2d(0, 0, 0));

            //score third specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scoreThirdSpecimen.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN -20),
                                    grabber.grabberIn()
                            ),
                            grabber.grabberMiddle(),
                            new SleepAction(0.3),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            //park
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberOut(),
                            new ParallelAction(
                                    parkInObservation.build(),
                                    arm.liftToTargetPosition(0),
                                    grabber.grabberRelease()
                            ),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.3)
                    )
            );

        }
    }
}