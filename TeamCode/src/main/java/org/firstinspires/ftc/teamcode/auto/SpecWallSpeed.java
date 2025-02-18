package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleSpec;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSpecWall;
import org.firstinspires.ftc.teamcode.commandBase.PrepareSpecDeposit;
import org.firstinspires.ftc.teamcode.commandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.commandBase.SpecPark;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.LowPrecisionFollow;
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

@Autonomous
public class SpecWallSpeed extends LinearOpMode {
    MotionPlannerEdit follower; //-17.7, 14.65
    MotionPlannerEdit lowPrecisionFollower;
    double movementPower = 0.8;
    ElapsedTime timer;
    Point spike1 = new Point(-24.25, 29.45); // 19600
    Point spike2 = new Point(-23.4, 39.0); // 19600
    Point spike3 = new Point(-23.2, 48.6); // 164.8 19632
    Point obsZone =  new Point(-11.0, 29.5); // -18.5, 26
    Point parkPoint = new Point(-10, 40);
    Point chamber1 = new Point(-30.2, -11);
    Point chamber2 = new Point(-32.75, -5.5);
    Point chamber3 = new Point(-33.5, 1);
    Point chamber4 = new Point(-32.5, 1);
    Point chamber5 = new Point(-31.205, 1);

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        timer = new ElapsedTime();
        follower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);
        lowPrecisionFollower = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);
        lowPrecisionFollower.setPermissibleTranslationalError(4);
        lowPrecisionFollower.setPermissibleHeadingError(5);
        lowPrecisionFollower.setMovementPower(0.98);
        Bezier preloadPath = new Bezier(
                0,
                new Point(0,0),
                chamber1);
        Bezier spike1Path = new MergedBezier(
                new Bezier(
                        0,
                        new Point(5, 0)
                ),
                new Bezier(
                        140,
                        new Point(-20, 15),
                        new Point(-10, 30),
                        spike1
                )
        );

        Bezier spike2Path = new Bezier(
                140,
                spike1Path.getEndPoint(),
                spike2
        );
        Bezier spike3Path = new Bezier(
                140,
                spike2Path.getEndPoint(),
                spike3
        );

        Bezier spikeToObs = new Bezier(
                0,
                spike1,
                obsZone
        );


//        Bezier chamberToObs = new MergedBezier(
//                40,
//                new Bezier(
//                        0,
//                        new Point(-5, 0)
//                ),
//                new Bezier(
//                        180,
//                        new Point(-20, 15),
//                        new Point(-10, 25),
//                        obsZone
//                )
//        );

        Bezier chamberToObs = new MergedBezier(
                0,
                new Bezier(
                        0,
                        chamber2,
                        new Point(-28, 0)
                ),
                new Bezier(
                        0,
                        new Point(-28, 0),
                        new Point(-28, 22)
                ),
                new Bezier(
                        new Point(-28, 22),
                        obsZone
                )
        );
        Bezier park = new MergedBezier(
                40,
                new Bezier(
                        0,
                        new Point(-5, 0)
                ),
                new Bezier(
                        180,
                        new Point(-20, 15),
                        new Point(-10, 30),
                        parkPoint
                )
        );

        Bezier obsToChamber = new Bezier(
                0,
                obsZone,
                new Point(-17, 5),
                new Point(chamber2.getX()-1.9, chamber2.getY())
        );
//        Bezier obsToChamber = new MergedBezier(
//                0,
//                new Bezier(
//                        obsZone,
//                        new Point(-18, 5)
//                ),
//                new Bezier(
//                        new Point(-18, 5),
//                        chamber2
//                )
//        );

//        Bezier obsToChamber2 = new MergedBezier(
//                0,
//                new Bezier(
//                        obsZone,
//                        new Point(-18, 5)
//                ),
//                new Bezier(
//                        new Point(-18, 5),
//                        new Point(chamber2.getX(), -2)
//                )
//        );



        Bezier obsToChamber2 = new Bezier(
                0,
                obsZone,
                new Point(-17, 5),
                new Point(chamber2.getX()-2.2, -3.5)
        );

//        Bezier obsToChamber3 = new MergedBezier(
//                0,
//                new Bezier(
//                        obsZone,
//                        new Point(-18, 5)
//                ),
//                new Bezier(
//                        new Point(-18, 5),
//                        new Point(chamber2.getX()-1, 1)
//                )
//        );

        Bezier obsToChamber3 = new Bezier(
                0,
                obsZone,
                new Point(-17, 5),
                new Point(chamber2.getX()-3, 1)
        );

//        Bezier obsToChamber4 = new MergedBezier(
//                0,
//                new Bezier(
//                        obsZone,
//                        new Point(-18, 5)
//                ),
//                new Bezier(
//                        new Point(-18, 5),
//                        new Point(chamber2.getX()-1.2, 4.2)
//                )
//        );

        Bezier obsToChamber4 = new Bezier(
                0,
                obsZone,
                new Point(-17, 5),
                new Point(chamber2.getX()-3.1, 4.5)
        );

        SequentialCommand preloadAndSpikes = new SequentialCommand(
//                new ParallelCommand(
//                        new PrepareSpecDeposit(),
//                        new SequentialCommand(
//                                new Wait(600),
//                                new FollowTrajectory(follower, preloadPath)
//                        )
//                ),
                new PrepareSpecDeposit(),
                new FollowTrajectory(follower, preloadPath),

                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new RunCommand(()->Pika.outtakeSlides.setMaxPower(1)),
                        new FollowTrajectory(follower, spike1Path),
                        new SequentialCommand(
                                new Wait(400),
                                new IntakeSampleTeleOp(),
                                new Wait(80),
                                new SlidesMove(19000),
                                new RunCommand(()-> Pika.newClaw.setPivotOrientation(140))
                        )
                ),

                new Grab(),
                new ParallelCommand(
                        new RunCommand(()-> follower.pause()),
                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(65, spike1))
                ),

                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(()-> follower.resume()),

                        new FollowTrajectory(follower, spike2Path),
                        new SequentialCommand(
                                new IntakeSampleTeleOp()
//                                new SlidesMove(10000)
                        )
                ),
                new RunCommand(()-> Pika.newClaw.setPivotOrientation(140)),
                new ParallelCommand(
                        new SlidesMove(20000)
                ),
                new Grab(),
                new ParallelCommand(
                        new SlidesMove(14000),
                        new RunCommand(()-> follower.pause()),
                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(50, spike2))
                ),

                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(()-> follower.resume()),
                        new FollowTrajectory(follower, spike3Path),
                        new SequentialCommand(
                                new IntakeSampleTeleOp()
//                                new SlidesMove(10000)
                        )

                ),
                new RunCommand(()-> Pika.newClaw.setPivotOrientation(140)),

                new ParallelCommand(
                        new SlidesMove(21000)
                ),
                new Grab(),
                new ParallelCommand(
                        new SlidesMove(8000),
                        new RunCommand(()-> follower.pause()),
                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(30, spike2))
                ),
                new SlidesMove(16000),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),


                new ParallelCommand(
                        new RunCommand(()-> follower.resume()),
                        new FollowTrajectory(follower, spikeToObs),
                        new IntakeSpecWall()

                ),

                new Wait(100),
//                new SlidesMove(12000),
                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_RETRACT.getPosition()))

                ),
                new Wait(150),


                new ParallelCommand(
                        new PrepareSpecDeposit(),
                        new FollowTrajectory(follower, obsToChamber),
                        new RunCommand(()-> follower.setMovementPower(movementPower))
                ),


                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),


                new ParallelCommand(
                        new IntakeSpecWall(),
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new FollowTrajectory(follower, chamberToObs)
                ),

                new Wait(100),
//                new SlidesMove(12000),
                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_RETRACT.getPosition()))

                ),
                new Wait(150),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new PrepareSpecDeposit(),
                        new FollowTrajectory(follower, obsToChamber2)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new FollowTrajectory(follower, chamberToObs),
                        new IntakeSpecWall()
                ),

                new Wait(100),
//                new SlidesMove(12000),
                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_RETRACT.getPosition()))

                ),
                new Wait(150),


                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new PrepareSpecDeposit(),
                        new FollowTrajectory(follower, obsToChamber3)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new FollowTrajectory(follower, chamberToObs),
                        new IntakeSpecWall()
                ),


                new Wait(100),
//                new SlidesMove(12000),

                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_RETRACT.getPosition()))

                ),
                new Wait(150),


                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new PrepareSpecDeposit(),
                        new FollowTrajectory(follower, obsToChamber4)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new RunCommand(()-> follower.setMovementPower(movementPower)),
                        new FollowTrajectory(follower, park),
                        new SpecPark()
                )
        );
        follower.pause();
        Pika.arm.setMaxMovementPower(0.96);
        Pika.outtakeSlides.setMaxPower(0.9);
        follower.setMovementPower(movementPower); //0.86
        waitForStart();

        preloadAndSpikes.init();
        while (opModeIsActive() && !isStopRequested()) {
            Pika.arm.update();
            Pika.localizer.update();
            if (Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            preloadAndSpikes.update();

            follower.update();
//
//            telemetry.addData("MP: ", follower.isEnd());
//            telemetry.addData("Loop speed: ", timer.milliseconds());
//            telemetry.update();
////
//            timer.reset();
        }
    }

}
