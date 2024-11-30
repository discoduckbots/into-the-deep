package org.firstinspires.ftc.teamcode.auto.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;


@Config
@Autonomous(name = "ArmAndLiftTest", group = "Test")
public class ArmAndLiftTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }
        Action scoreHighAction = new ParallelAction(
                wrist.wristFoldOutAction(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );

        Action robotTravelActionTest = new SequentialAction(
                arm.armVerticalAction()
                ,lift.liftDownAction()
                ,arm.armRobotTravelAction()
        );

        Action foldBackAction = new ParallelAction(
                arm.armfoldbackaction(),
                lift.liftDownAction()
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        scoreHighAction
                        ,new SleepAction(1)
                        ,robotTravelActionTest
                        ,new SleepAction(1)
                        ,foldBackAction
                ));
    }



}
