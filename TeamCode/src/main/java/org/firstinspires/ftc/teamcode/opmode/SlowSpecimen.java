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
@Autonomous(name = "Slow_Specimen", group = "Autonomous")
public class SlowSpecimen extends DuckbotAuto {

    MecanumDrive drive = null;
    Grabber teleGrabber = null;

    Intake teleIntake = null;

    private static final TranslationalVelConstraint SLOW_VEL= new TranslationalVelConstraint(60);
    private static final ProfileAccelConstraint SLOW_ACC = new ProfileAccelConstraint(-30, 60);

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

        TrajectoryActionBuilder driveToBarPreload = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(27, 16)); // move to bar

        TrajectoryActionBuilder scorePreload = driveToBarPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, 16));

        TrajectoryActionBuilder pushFirstSample = driveToBarPreload.endTrajectory().fresh()
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(22, -15), Math.toRadians(-90), SLOW_VEL, SLOW_ACC) //back up from bar
                .splineToConstantHeading(new Vector2d(50, -38), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to first sample
                .splineToConstantHeading(new Vector2d(15, -38), Math.toRadians(0), SLOW_VEL, SLOW_ACC); //first move to wall

        TrajectoryActionBuilder pushSecondSample = pushFirstSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(50, -44.5), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to second sample
                .splineToConstantHeading(new Vector2d(15, -46.5), Math.toRadians(0), SLOW_VEL, SLOW_ACC); //second move to wall

        TrajectoryActionBuilder pushThirdSample = pushSecondSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(50, -52), Math.toRadians(180), SLOW_VEL, SLOW_ACC) //move to third sample
                .splineToConstantHeading(new Vector2d(15, -52), Math.toRadians(0), SLOW_VEL, SLOW_ACC); //third move to wall, radians was 90

        TrajectoryActionBuilder grabFirstSpecimen = pushThirdSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(15, -30), Math.toRadians(180), SLOW_VEL, SLOW_ACC)
                .strafeTo(new Vector2d(.75, -26), SLOW_VEL, SLOW_ACC);

        TrajectoryActionBuilder scoreFirstSpecimen = grabFirstSpecimen.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(28, 10), 0, SLOW_VEL, SLOW_ACC)
                .strafeTo(new Vector2d(33, 14), SLOW_VEL, SLOW_ACC);

        TrajectoryActionBuilder secondBackUpLittle = scoreFirstSpecimen.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(22, -15), Math.toRadians(-90), SLOW_VEL, SLOW_ACC);

        TrajectoryActionBuilder grabSecondSpecimen = secondBackUpLittle.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(15, -30), Math.toRadians(180), SLOW_VEL, SLOW_ACC)
                .strafeTo(new Vector2d(.2, -26), SLOW_VEL, SLOW_ACC);

        TrajectoryActionBuilder scoreSecondSpecimen = grabSecondSpecimen.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(28, 5.8), 0, SLOW_VEL, SLOW_ACC)
                .strafeTo(new Vector2d(35, 5.8), SLOW_VEL, SLOW_ACC);


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

            // score preload
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    driveToBarPreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR)
                            ),
                            new ParallelAction(
                                    scorePreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                            ),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            // push samples
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    arm.liftToTargetPosition(12),
                                    grabber.grabberOut(),
                                    pushFirstSample.build()

                            ),
                            new ParallelAction(
                                    grabber.grabberRelease(),
                                    pushSecondSample.build()
                            )

                    )
            );
            //score first specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    grabFirstSpecimen.build()
                            ),
                            grabber.grabberGrab(),
                            new SleepAction(0.3),
                            new ParallelAction(
                                    scoreFirstSpecimen.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                                    grabber.grabberIn()
                            ),
                            new SleepAction(.35),
                            arm.liftToTargetPosition(Arm.LIFT_EXTRA_ABOVE),
                            new SleepAction(1),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )

            );

            //grab and score second specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    grabber.grabberGrab(),
                                    new SleepAction(.35),
                                    grabber.grabberOut(),
                                    secondBackUpLittle.build(),
                                    arm.liftToTargetPosition(12)
                            ),
                            new ParallelAction(
                                    grabSecondSpecimen.build(), //spline to grab wall pos
                                    grabber.grabberRelease()
                            ),
                            grabber.grabberGrab(),
                            new SleepAction(0.3),
                            new ParallelAction(
                                    scoreSecondSpecimen.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                                    grabber.grabberIn()
                            ),
                            arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR),
                            new SleepAction(0.65),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )

            );



        }
    }
}
