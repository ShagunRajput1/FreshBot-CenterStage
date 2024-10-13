package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.core.Jerry;
import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.pathing.RobotPose;

@Autonomous
public class MainAuto extends LinearOpMode {
    public enum StartPos {
        BUCKET,
        CHAMBER
    }
    StartPos startPos;
    Pose3D robotPose;
    Point chamber = new Point(0, 0);
    Point sample1 = new Point(0, 0);
    Point sample2 = new Point(0, 0);
    Point sample3 = new Point(0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        Jerry.init(hardwareMap);
        ElapsedTime timer;
        timer = new ElapsedTime();
        boolean start = false;
        waitForStart();
        while (opModeIsActive()) {
            if (!start) {
                timer.reset();
                start = true;
            }
            if (timer.seconds() <= 1.2) {
                Jerry.drivetrain.drive(1, -90, 0, 0.5);
            }
            else {
                Jerry.drivetrain.drive(0, 0, 0, 0);
            }

        }

    }
}
