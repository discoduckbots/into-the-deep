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
@Autonomous(name = "Spline Sample Auto", group = "Autonomous")
public class SplineSampleAuto extends DuckbotAuto {

    private Grabber teleGrabber = null;
    private Intake teleIntake = null;
    private AutoArm arm = null;
    private AutoIntake intake = null;
    private AutoGrabber grabber = null;
    MecanumDrive drive = null;

    private static final TranslationalVelConstraint SLOW_VEL= new TranslationalVelConstraint(60);
    private static final ProfileAccelConstraint SLOW_ACC = new ProfileAccelConstraint(-30, 60);


    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = new AutoArm(hardwareStore);
        intake = new AutoIntake(hardwareStore);
        grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();

        teleGrabber = hardwareStore.getGrabber();
        teleIntake = hardwareStore.getIntake();
        MecanumDrive drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(5.17, 23.09), Math.toRadians(-45));

        TrajectoryActionBuilder grabFirstSample = scorePreload.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(11.5, 10, Math.toRadians(0)), Math.toRadians(180), SLOW_VEL, SLOW_ACC);
                //.strafeTo(new Vector2d(0, 0));

        TrajectoryActionBuilder scoreFirstSample = grabFirstSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(4.5, 20, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder grabSecondSample = scoreFirstSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(11.9, 25.2, Math.toRadians(0)), Math.toRadians(180), SLOW_VEL, SLOW_ACC);
                //.strafeTo(new Vector2d(0, 0));

        TrajectoryActionBuilder scoreSecondSample = grabSecondSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(4, 21.5, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder moveToThirdSample = scoreSecondSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(37.7, 15.5, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder grabThirdSample = moveToThirdSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, 22), SLOW_VEL, SLOW_ACC);

        /*TrajectoryActionBuilder moregrabThirdSample = grabThirdSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, 26.5), SLOW_VEL, SLOW_ACC);
*/
        TrajectoryActionBuilder lessgrabThirdSample = grabThirdSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, 17), SLOW_VEL, SLOW_ACC);

        TrajectoryActionBuilder scoreThirdSample = lessgrabThirdSample.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(4, 24, Math.toRadians(-45)), Math.toRadians(-45));





        Actions.runBlocking(
                new SequentialAction(
                        grabber.grabberGrab(),
                        grabber.grabberMiddle(),
                        intake.intakeOpen()
                )
        );

        teleIntake.rotateIntakeTo0();

        waitForStart();


        if (opModeIsActive()) {
            if (isStopRequested()) return;

            //Score Preload
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            new ParallelAction(
                                    scorePreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                    ),
                            new SleepAction(0.5),
                            grabber.grabberOut(),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            new SleepAction(0.5)
                    )
            );

            //grab first block
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberIn(),
                            new ParallelAction(
                                    grabFirstSample.build(),
                                    arm.liftToTargetPosition(0),
                                    intake.extend(1.0)
                            ),
                            intake.intakeDown(),
                            grabber.grabberRelease(),
                            new SleepAction(0.8),
                            intake.intakeClose(),
                            new SleepAction(0.8),
                            new ParallelAction(
                                    intake.intakeUp(),
                                    intake.retract()
                            ),
                            new SleepAction(0.5),
                            grabber.grabberGrab(),
                            new SleepAction(0.5)
                    )
            );

            //Score first block
            Actions.runBlocking(
                   new SequentialAction(
                           new ParallelAction(
                                   scoreFirstSample.build(),
                                   new SequentialAction (
                                           intake.intakeOpen(),
                                           arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                   )
                           ),
                           grabber.grabberOut(),
                           new SleepAction(1),
                           grabber.grabberRelease(),
                           new SleepAction(0.5)
                   )
            );

            //grab second block
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberIn(),
                            new ParallelAction(
                                    grabSecondSample.build(),
                                    arm.liftToTargetPosition(0),
                                    intake.extend(1.0)
                            ),
                            intake.intakeDown(),
                            grabber.grabberRelease(),
                            new SleepAction(0.8),
                            intake.intakeClose(),
                            new SleepAction(0.8),
                            new ParallelAction(
                                    intake.intakeUp(),
                                    intake.retract()
                            ),
                            new SleepAction(0.5),
                            grabber.grabberGrab(),
                            new SleepAction(0.5)
                    )
            );

            //Score second block
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scoreSecondSample.build(),
                                    new SequentialAction (
                                            intake.intakeOpen(),
                                            arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                    )
                            ),
                            grabber.grabberOut(),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            new SleepAction(0.5)
                    )
            );

            //Grab third block
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberIn(),
                            new ParallelAction(
                                    moveToThirdSample.build(),
                                    arm.liftToTargetPosition(0),
                                    intake.intakeRotate(1.0), //rotate90
                                    intake.intakeDown() //intakeWonky
                            ),
                            grabThirdSample.build(),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            intake.intakeClose(),
                            new SleepAction(.5),
                            lessgrabThirdSample.build(),
                            new SleepAction(.2),
                            new ParallelAction(
                                    intake.intakeUp(),
                                    intake.intakeRotate(0.0)
                            ),
                            new SleepAction(0.8),
                            grabber.grabberGrab(),
                            new SleepAction(0.5)
                    )
            );

            //Score third sample
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    scoreThirdSample.build(),
                                    new SequentialAction (
                                            intake.intakeOpen(),
                                            arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                    )
                            ),
                            grabber.grabberOut(),
                            new SleepAction(0.8),
                            grabber.grabberRelease(),
                            new SleepAction(0.8),
                            arm.liftToTargetPosition(0)
                    )
            );


        }
    }}