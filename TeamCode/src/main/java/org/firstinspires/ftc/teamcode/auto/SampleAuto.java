package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeAuto;
import org.firstinspires.ftc.teamcode.commandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
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
    public static Point bucketDeposit = new Point(11.8, 7.5);


    public static Point submersible = new Point(45, -20);
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        follower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);

        while (opModeInInit()) {
            bucket = new Bezier(-45,        // If heading is changed, make sure to change the setHeading later
                    new Point(0, 0),
                    bucketDeposit);
            sample1 = new Bezier(0,
                    bucketDeposit,
                    new Point(13.89, 7.7209)
            );
            sample2 = new Bezier(0,
                    bucketDeposit,
                    new Point(12.2966, 18)
            );
            sample3 = new Bezier(14.46,
                    bucketDeposit,
                    new Point(15.10, 22.4));

            submersibleIntake = new Bezier(-90,
                    bucketDeposit,
                    new Point(44, 3.2),
                    new Point(45.9, -12.3),
                    submersible);




        }

        SequentialCommand commandRunner = new SequentialCommand(
                new ParallelCommand(new FollowTrajectory(follower, bucket),
                                    new SequentialCommand(
                                            new Wait(600),
                                            new PrepareOuttakeAuto()
                                    )
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new ParallelCommand(
                    new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                    new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                    new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                    new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                    new RunCommand(()->Pika.localizer.setHeading(-45))

                ),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample1)
                ),
                new ParallelCommand(
                        new SlidesMove(21500),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),

                new Grab(),


                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                                    bucketDeposit)),
                                new PrepareOuttakeAuto()
                        ),

                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                        new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                        new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                        new RunCommand(()->Pika.localizer.setHeading(-45))

                ),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample2)
                ),
                new ParallelCommand(
                        new SlidesMove(24510),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                    ),


                new Grab(),

                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                        bucketDeposit)),
                        new PrepareOuttakeAuto()
                ),

                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                        new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                        new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                        new RunCommand(()->Pika.localizer.setHeading(-45))
                ),
                new ParallelCommand(
                        new IntakeSample(),
                        new FollowTrajectory(follower, sample3)
                ),
                new ParallelCommand(
                        new SlidesMove(19600),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),
                new Grab(),
                new ParallelCommand(new FollowTrajectory(follower, new Bezier(-45,
                        bucketDeposit)),
                        new SequentialCommand(
                                new Wait(300),
                                new PrepareOuttakeAuto()
                        )
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                        new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                        new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                        new RunCommand(()->Pika.localizer.setHeading(-45))
                ),
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
