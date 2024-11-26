package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlanner;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.pathing.PathPoint;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.pathing.SimpleFollower;
import org.firstinspires.ftc.teamcode.pathing.SimplePath;
import org.opencv.dnn.Model;

@Autonomous
public class PathFollowerTest extends LinearOpMode {
    Bezier testPath;
    Bezier path2;
    Bezier path3;
    Bezier bucket;
    Bezier sample1;
    Bezier sample2;
    MotionPlannerEdit follower;
    public static Point bucketDeposit = new Point(15, 34.5);
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        follower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);

        while (opModeInInit()) {
            bucket = new Bezier(-45,
                    new Point(0, 0),
                    bucketDeposit);
            sample1 = new Bezier(0,
                    bucket.getEndPoint(),
                    new Point(25, 31.2)
            );
            sample2 = new Bezier(0,
                    bucket.getEndPoint(),
                    new Point(25, 41.5)
            );




        }

        SequentialCommand commandRunner = new SequentialCommand(
                new FollowTrajectory(follower, bucket),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new Wait(1000),
                new RunCommand(()->Pika.claw.setClaw(Claw.ClawPosition.OPEN.getPosition())),
                new Wait(1000),
                new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),
                new IntakeSample(),
                new FollowTrajectory(follower, sample1),
                new Grab(),
////
////                // Add something here
////
                new FollowTrajectory(follower, new Bezier(-45,
                    new Point(Pika.localizer.getX(), Pika.localizer.getY()),
                    bucketDeposit
                )),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.claw.setClaw(Claw.ClawPosition.OPEN.getPosition())),
                new Wait(5000),
                new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),
                new IntakeSample(),
                new FollowTrajectory(follower, sample2)

//                // Add something here
//
//                new FollowTrajectory(follower, new Bezier(-45,
//                        new Point(Pika.localizer.getX(), Pika.localizer.getY()),
//                        bucketDeposit
//                )),
//                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
//                new RunCommand(()->Pika.claw.setClaw(Claw.ClawPosition.OPEN.getPosition())),
//                new Wait(5000),
//                new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),

        );


        commandRunner.init();
        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            commandRunner.update();
            follower.update();
            Pika.arm.update();
            if (Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            else {
                Pika.outtakeSlides.freeMove();
            }
            telemetry.addData("", follower.getTelemetry());
            telemetry.addData("", Pika.arm.getTelemetry());
            telemetry.update();
            Pika.localizer.update();
        }
    }
}
