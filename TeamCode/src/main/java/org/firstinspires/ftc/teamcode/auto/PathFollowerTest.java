package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
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

@Autonomous
public class PathFollowerTest extends LinearOpMode {
    Bezier testPath;
    Bezier path2;
    Bezier path3;
    MotionPlannerEdit follower;
    @Override
    public void runOpMode() throws InterruptedException {
        Localizer localizer = new Localizer(this, hardwareMap);
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        follower = new MotionPlannerEdit(drivetrain, localizer, hardwareMap);

        while (opModeInInit()) {
            testPath = new Bezier(-90,
                    new Point(22, 10),
                    new Point(34, 18),
                    new Point(75, 18),
                    new Point(75, 20));

            path2 = new Bezier(45,
                    testPath.getEndPoint(),
                    new Point(60, -15));

            path3 = new Bezier(0,
                    path2.getEndPoint(),
                    new Point(0, 0));

        }

        SequentialCommand commandRunner = new SequentialCommand(
                new FollowTrajectory(follower, testPath),
//                new Wait(5000),
                new FollowTrajectory(follower, path2),
//                new Wait(2000),
                new FollowTrajectory(follower, path3)
        );
        commandRunner.init();
        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            commandRunner.update();
            follower.update();
            telemetry.addData("", follower.getTelemetry());
            telemetry.update();
            localizer.update();
        }
    }
}
