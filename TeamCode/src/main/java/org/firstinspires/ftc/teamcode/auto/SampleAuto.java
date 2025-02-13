package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.AlignWithSample;
import org.firstinspires.ftc.teamcode.commandBase.AutoIntakeFromSub;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeAuto;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeBasket;
import org.firstinspires.ftc.teamcode.commandBase.PreparePark;
import org.firstinspires.ftc.teamcode.commandBase.PrepareSpecDeposit;
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
import org.firstinspires.ftc.teamcode.pathing.MergedBezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.teleop.SohamsRobot;

@Autonomous
public class SampleAuto extends LinearOpMode {
    Bezier bucket, sample1, sample2, sample3, submersibleIntake, parkPath;
    ElapsedTime timer = new ElapsedTime();
    MotionPlannerEdit follower;
    public static Point bucketDeposit = new Point(10.3, 11.2);
    public static Point subBucketDeposit = new Point(9.45, 12); //9.75, 11.65
    Bezier subToBucket = new MergedBezier(
            -45,
            new Bezier(
                    -45,
                    submersible,
                    new Point(50, -14)
            ),
            new Bezier(
                    new Point(50, -14),
                    subBucketDeposit
            )
    );



    public static boolean subCycleDone = false;
    private boolean emergencyParking = false;


    public static Point submersible = new Point(50, -18.25);
    public static Point parkPoint = new Point(45, -23.5);

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
                new Point(13.89, 7)
        );
        sample2 = new Bezier(0,
                bucketDeposit,
                new Point(12.2966, 16)
        );
        sample3 = new Bezier(14.6,
                bucketDeposit,
                new Point(15.10, 21.25));

        submersibleIntake = new MergedBezier(
                -90,
                new Bezier(
                        bucketDeposit,
                        new Point(50, 3)
                ),
                new Bezier(
                        new Point(50, 3),
                        submersible
                )
//                bucketDeposit,
//                new Point(44, 3.2),
//                new Point(45.9, -12.3),
//                submersible
        );
        parkPath = new MergedBezier(
                -90,
                new Bezier(
                        bucketDeposit,
                        new Point(48, 0)
                ),
                new Bezier(
                        new Point(48, 0),
                        parkPoint
                )
//                bucketDeposit,
//                new Point(44, 3.2),
//                new Point(45.9, -12.3),
//                submersible
        );







        SequentialCommand zeroPlusFour = new SequentialCommand(
                new ParallelCommand(
                        new FollowTrajectory(follower, bucket),
                        new SequentialCommand(
                                new Wait(800),
                                new PrepareOuttakeBasket()
                        )
                ),
                new Wait(100),
                new RunCommand(()->follower.pause()),
                new PrepareOuttakeAuto(),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
//                    new RunCommand(()->Pika.localizer.setX(bucketDeposit.getX())),
//                    new RunCommand(()->Pika.localizer.setY(bucketDeposit.getY())),
//                    new RunCommand(()->Pika.localizer.setHeading(-45))

                ),
                new ParallelCommand(
                        new IntakeSampleTeleOp(),
                        new SequentialCommand(
                                new FollowTrajectory(follower, sample1),
                                new Wait(80)
                        )
                ),
                new ParallelCommand(
                        new SlidesMove(21500),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),

                new Grab(),


                new ParallelCommand(
                        new SequentialCommand(
                                new FollowTrajectory(follower, new Bezier(-45,
                                        bucketDeposit)),
                                new RunCommand(()->follower.pause())
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
                new RunCommand(()->follower.resume()),

                new ParallelCommand(
                        new IntakeSampleTeleOp(),
                        new SequentialCommand(
                                new FollowTrajectory(follower, sample2),
                                new Wait(80)
                        )
                ),
                new ParallelCommand(
                        new SlidesMove(24500),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),


                new Grab(),

                new ParallelCommand(
                        new SequentialCommand(
                                new FollowTrajectory(follower, new Bezier(-45,
                                        bucketDeposit)),
                                new RunCommand(()->follower.pause())
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
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new IntakeSampleTeleOp(),
                        new SequentialCommand(
                                new FollowTrajectory(follower, sample3),
                                new Wait(80)
                        )
                ),
                new ParallelCommand(
                        new SlidesMove(19000),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),
                new Grab(),
                new ParallelCommand(
                        new SequentialCommand(
                                new FollowTrajectory(follower, new Bezier(-45,
                                        bucketDeposit)),
                                new RunCommand(()->follower.pause())
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
                new RunCommand(()->follower.resume()),
                new ParallelCommand(
                        new IntakeSampleTeleOp(),
                        new FollowTrajectory(follower, submersibleIntake)
                )
        );


        SequentialCommand subCycles = new SequentialCommand(
                new AutoIntakeFromSub(follower),
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                new ParallelCommand(
                        new FollowTrajectory(follower, subToBucket)
//                        new SequentialCommand(
//                                new Wait(800),
//                                new PrepareOuttakeBasket()
//                        )
                ),
                new ParallelCommand(
                        new PrepareOuttakeAuto(),
                        new RunCommand(()->follower.pause())
                ),
                new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks()),
                new RunCommand(()->follower.resume()),
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

        SequentialCommand emergencyPark = new SequentialCommand(
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DETECT.getPosition())),
                new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                new ParallelCommand(
                        new SequentialCommand(
                                new RunCommand(()-> Pika.outtakeSlides.resume()),
                                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks())
                        ),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.RETRACT.getPosition()))
                ),
                new Wait(200),
                new PreparePark(),
                new FollowTrajectory(follower, parkPath)
        );


        waitForStart();
        zeroPlusFour.init();
        timer.reset();




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
                follower.setMovementPower(0.85);
                subCycleDone = true;
                subCycles.init();
            }

            if (timer.seconds() >= 25 && subCycleDone && subCycles.getIndex() == 0 && !emergencyParking) {
                subCycles.stop();
                zeroPlusFour.stop();
                emergencyPark.init();
                emergencyParking = true;
            }

//            telemetry.addData("Timer: ", timer.seconds());
//            telemetry.addData("Slides", Pika.outtakeSlides.getTelemetry());
//            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
//            telemetry.addData("EmergencyPark", emergencyPark.getIndex());
//            telemetry.addData("EPark: ", emergencyPark.isFinished());
//            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
            subCycles.update();
            emergencyPark.update();
            Pika.localizer.update();
        }
    }
}

// X: 8.9, y: 32.22, 42.2
//SlidePos1: 31582, 36767