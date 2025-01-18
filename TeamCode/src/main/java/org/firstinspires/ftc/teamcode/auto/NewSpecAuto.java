package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleSpec;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.PrepareSpecDeposit;
import org.firstinspires.ftc.teamcode.commandBase.SlidesMove;
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
public class NewSpecAuto extends LinearOpMode {
    MotionPlannerEdit follower; //-17.7, 14.65
    MotionPlannerEdit lowPrecisionFollower;
    ElapsedTime timer;
    Point spike1 = new Point(-23.8, 29.45); // 19600
    Point spike2 = new Point(-23.8, 39.15); // 19600
    Point spike3 = new Point(-24.8, 45.75); // 164.8 19632
    Point obsZone =  new Point(-17, 27);
    Point parkPoint = new Point(-10, 34);
    Point chamber1 = new Point(-28.65, -5);
    Point chamber2 = new Point(-30.25, -3);
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
        lowPrecisionFollower.setMovementPower(0.9);
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
                        -320,
                        spike1,
                        obsZone
                );


        Bezier chamberToObs = new MergedBezier(
                40,
                new Bezier(
                        0,
                        new Point(-5, 0)
                ),
                new Bezier(
                        180,
                        new Point(-20, 15),
                        new Point(-10, 30),
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
                        new Point(-10, 7),
                        chamber2
        );

        Bezier obsToChamber2 = new Bezier(
                0,
                obsZone,
                new Point(-10, 7),
                new Point(chamber2.getX(), 0)
        );
        Bezier obsToChamber3 = new Bezier(
                0,
                obsZone,
                new Point(-10, 7),
                new Point(chamber2.getX(), 2)
        );

        SequentialCommand preloadAndSpikes = new SequentialCommand(

                new PrepareSpecDeposit(),
                new FollowTrajectory(follower, preloadPath),

                new Wait(200),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new FollowTrajectory(follower, spike1Path),
                        new SequentialCommand(
                                new Wait(500),
                                new IntakeSampleTeleOp()
                        )
                ),
                new ParallelCommand(
                        new SlidesMove(19000),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(140))
                ),
                new Grab(),
                new ParallelCommand(
                        new RunCommand(()-> follower.pause()),
                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(55, spike1))
                ),

                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(()-> follower.resume()),

                        new FollowTrajectory(follower, spike2Path),
                        new IntakeSampleTeleOp()
                ),
                new ParallelCommand(
                        new SlidesMove(20000),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(140))
                ),
                new Grab(),
                new ParallelCommand(
                        new RunCommand(()-> follower.pause()),
                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(30, spike2))
                ),
                new Wait(100),

//                new ParallelCommand(
//                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
//                        new RunCommand(()-> follower.resume()),
//                        new FollowTrajectory(follower, spike3Path),
//                        new IntakeSampleTeleOp()
//                ),
//                new ParallelCommand(
//                        new SlidesMove(21000),
//                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(140))
//                ),
//                new Grab(),
//                new ParallelCommand(
//                        new SlidesMove(6000),
//                        new RunCommand(()-> follower.pause()),
//                        new LowPrecisionFollow(lowPrecisionFollower, new Bezier(0, spike2))
//                ),


                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(()-> follower.resume()),
                        new FollowTrajectory(follower, spikeToObs),
                        new IntakeSampleSpec()

                ),


                new Wait(800),
                new SlidesMove(17000),
                new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new PrepareSpecDeposit(),
                new ParallelCommand(
                        new FollowTrajectory(follower, obsToChamber)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                new ParallelCommand(
                        new FollowTrajectory(follower, chamberToObs),
                        new IntakeSampleSpec()
                ),

                new Wait(800),
                new SlidesMove(17000),
                new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new PrepareSpecDeposit(),
                new ParallelCommand(
                        new FollowTrajectory(follower, obsToChamber2)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),

                new ParallelCommand(
                        new FollowTrajectory(follower, chamberToObs),
                        new IntakeSampleSpec()
                ),

                new Wait(800),
                new SlidesMove(17000),
                new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new PrepareSpecDeposit(),
                new ParallelCommand(
                        new FollowTrajectory(follower, obsToChamber3)
                ),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                new ParallelCommand(
                        new FollowTrajectory(follower, park),
                        new IntakeSampleSpec()
                )

//                new Wait(1000),
//                new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
//                new ParallelCommand(
//                        new PrepareSpecDeposit(),
//                        new FollowTrajectory(follower, obsToChamber)
//                ),
//                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
//                new ParallelCommand(
//                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
//                        new FollowTrajectory(follower, chamberToObs),
//                        new IntakeSampleSpec()
//                )
        );
        follower.pause();
        follower.setMovementPower(0.8);
        waitForStart();

        preloadAndSpikes.init();
        while (opModeIsActive() && !isStopRequested()) {
            Pika.arm.update();
            Pika.localizer.update();
            if (Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            preloadAndSpikes.update();

            follower.update();

//            telemetry.addData("MP: ", follower.getTelemetry());
//            telemetry.addData("Loop speed: ", timer.milliseconds());
//            telemetry.update();
//
//            timer.reset();
        }
    }

}
