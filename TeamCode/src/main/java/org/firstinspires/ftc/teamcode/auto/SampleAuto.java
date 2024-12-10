package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandBase.ArmMove;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttake;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeAuto;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
import org.firstinspires.ftc.teamcode.commandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.pathing.Point;

@Autonomous
public class SampleAuto extends LinearOpMode {
    Bezier bucket, sample1, sample2, sample3, submersibleIntake;
    MotionPlannerEdit follower;
    public static Point bucketDeposit = new Point(7.5, 14.5);

    public static Point submersible = new Point(49, -18);
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        follower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);

        while (opModeInInit()) {
            bucket = new Bezier(-45,
                    new Point(0, 0),
                    bucketDeposit);
            sample1 = new Bezier(0,
                    bucketDeposit,
                    new Point(11.6, 7.8)
            );
            sample2 = new Bezier(0,
                    bucketDeposit,
                    new Point(11.6, 17)
            );
            sample3 = new Bezier(13.5,
                    bucketDeposit,
                    new Point(10.76, 21.9));

            submersibleIntake = new Bezier(-90,
                    bucketDeposit,
                    submersible);




        }

        SequentialCommand commandRunner = new SequentialCommand(
                new ParallelCommand(new FollowTrajectory(follower, bucket),
                                    new PrepareOuttakeAuto()
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition())),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample1)
                ),
                new SlidesMove(24000),
                new Wait(2000),

                new Grab(),


                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                                    bucketDeposit)),
                                new PrepareOuttakeAuto()
                        ),

                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition())),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample2)
                ),
                new SlidesMove(24000),
                new Wait(2000),

                new Grab(),

                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                        bucketDeposit)),
                        new PrepareOuttakeAuto()
                ),

                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition())),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample3)
                ),
                new SlidesMove(25000),
                new Wait(2000),
                new Grab(),
                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                        bucketDeposit)),
                        new PrepareOuttakeAuto()
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition())),
                new IntakeSample()
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
            telemetry.addData("", Pika.outtakeSlides.getTelemetry());
            telemetry.update();
            Pika.localizer.update();
        }
    }
}

// X: 8.9, y: 32.22, 42.2
//SlidePos1: 31582, 36767
