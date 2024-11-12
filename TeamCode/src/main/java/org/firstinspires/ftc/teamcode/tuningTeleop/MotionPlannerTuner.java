package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
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

@Config
@Autonomous
public class MotionPlannerTuner extends LinearOpMode {

    public static double targetX, targetY;
    public static double targetHeading;

    public static double kX = 0.08, pX = 0.02, iX = 0.0015, dX = 0.00065;
    public static double kY = 0.14, pY = 0.03, iY, dY;
    public static double kT = 0.085, pT = 0.01, iT = 0.0001, dT;
    public static double pTr = 0.01, iTr = 0.0001, dTr;
    Bezier testPath;
    Bezier returnPath;
    MotionPlannerEdit follower;
    @Override
    public void runOpMode() throws InterruptedException {
        Localizer localizer = new Localizer(this, hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new MotionPlannerEdit(drivetrain, localizer, hardwareMap);
        follower.startTrajectory(new Bezier(
                new Point(0, 0))
        );
        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            if (follower.isFinished()) {
                follower.startTrajectory(new Bezier(
                        targetHeading,
                        new Point(localizer.getX(), localizer.getY()),
                        new Point(targetX, targetY)
                ));
            }
            follower.update();
            follower.setTerms(kX, kY, kT, pX, pY, pT, iX, iY, iT, dX, dY, dT, pTr, iTr, dTr);
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
            localizer.update();
        }
    }
}
