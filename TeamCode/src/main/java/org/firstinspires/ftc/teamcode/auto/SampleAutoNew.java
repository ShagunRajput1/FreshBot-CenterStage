package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandBase.AutoIntakeFromSub;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeAuto;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeBasket;
import org.firstinspires.ftc.teamcode.commandBase.PreparePark;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
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
public class SampleAutoNew extends LinearOpMode {
    Bezier bucket, sample1, sample2, sample3, submersibleIntake, parkPath;
    MotionPlannerEdit follower;
    public static Point bucketDeposit = new Point(11, 10);
    public static Point subBucketDeposit = new Point(10.85, 11);
    Bezier subToBucket = new Bezier(
            -45,
            submersible,
            new Point(45, 10),
            new Point(15, 10),
            subBucketDeposit
    );



    public static boolean subCycleDone = false;

    PreparePark park = new PreparePark();


    public static Point submersible = new Point(45, -18.8);
    public static Point parkPoint = new Point(45, -23);

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        subCycleDone = false;
        follower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);


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
        sample3 = new Bezier(14.6,
                bucketDeposit,
                new Point(15.10, 22.8));

        submersibleIntake = new Bezier(-90,
                bucketDeposit,
                new Point(44, 3.2),
                new Point(45.9, -12.3),
                submersible);
        parkPath = new Bezier(-90,
                bucketDeposit,
                new Point(44, 3.2),
                new Point(45.9, -12.3),
                parkPoint);







        SequentialCommand zeroPlusFour = new SequentialCommand(
                new ParallelCommand(
                        new FollowTrajectory(follower, bucket),
                        new SequentialCommand(
                                new Wait(700),
                                new PrepareOuttakeBasket()
                        )
                ),
                new Wait(100),
//                new RunCommand(()->Pika.localizer.pause()),
                new PrepareOuttakeAuto(),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.localizer.resume()),
                new ParallelCommand(
                    new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                    new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                    new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                    new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                    new RunCommand(()->Pika.localizer.setHeading(-45))

                ),
                new Wait(100),
                new SequentialCommand(
                        new FollowTrajectory(follower, sample1),
                        new IntakeSample(20500)
                ),

                new Grab(),


                new ParallelCommand(
                        new FollowTrajectory(follower, new Bezier(-45,
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
                new RunCommand(()->Pika.localizer.resume()),
                new Wait(100),

                new SequentialCommand(
                        new FollowTrajectory(follower, sample2),
                        new IntakeSample(24000)
                ),

                new Grab(),

                new ParallelCommand(
                        new SequentialCommand(
                                new FollowTrajectory(follower, new Bezier(-45,
                                        bucketDeposit))
//                                new RunCommand(()->Pika.localizer.pause())
                        ),
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
                new RunCommand(()->Pika.localizer.resume()),
                new Wait(100),
                new ParallelCommand(
                        new IntakeSampleTeleOp(),
                        new FollowTrajectory(follower, sample3)
                ),
                new ParallelCommand(
                        new SlidesMove(17000),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),
                new Grab(),

                new ParallelCommand(
                        new SequentialCommand(
                                new FollowTrajectory(follower, new Bezier(-45,
                                        bucketDeposit))
//                                new RunCommand(()->Pika.localizer.pause())
                        ),
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
                new RunCommand(()->Pika.localizer.resume()),
                new Wait(100),
                new ParallelCommand(
                        new RetractAll(),
                        new FollowTrajectory(follower, submersibleIntake)
                )
        );


        SequentialCommand subCycles = new SequentialCommand(
                new AutoIntakeFromSub(follower),
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                new ParallelCommand(
                        new FollowTrajectory(follower, subToBucket),
                        new SequentialCommand(
                                new PrepareOuttakeBasket(),
                                new Wait(800)
                        )
                ),
                new ParallelCommand(
                        new PrepareOuttakeAuto()
//                        new RunCommand(()->Pika.localizer.pause())
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->Pika.localizer.resume()),
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                        new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                        new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                        new RunCommand(()->Pika.localizer.setHeading(-45))
                ),

                new ParallelCommand(
                    new FollowTrajectory(follower, parkPath),
                    new SequentialCommand(
                            new Wait(100),
                            new PreparePark()
                    )
                )

        );

        follower.setMovementPower(0.95);
        zeroPlusFour.init();
        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            zeroPlusFour.update();

            follower.update();
            Pika.arm.update();
            if (Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            else {
                Pika.outtakeSlides.freeMove();
            }

            if (zeroPlusFour.isFinished() && !subCycleDone) {
                follower.setMovementPower(0.9);
                subCycleDone = true;
                subCycles.init();
            }

            subCycles.update();
            telemetry.addData("",follower.getTelemetry());
            telemetry.addData("", Pika.limelight.getTelemetry());
            telemetry.addData("CommandRunner: ", zeroPlusFour.isFinished());
            telemetry.addData("SubCycle: ", subCycles.isFinished());
            telemetry.update();
            Pika.localizer.update();
        }
        Pika.limelight.stop();

    }
}

// X: 8.9, y: 32.22, 42.2
//SlidePos1: 31582, 36767
