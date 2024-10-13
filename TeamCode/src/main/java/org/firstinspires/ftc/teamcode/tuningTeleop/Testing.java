package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Jerry;

@TeleOp(name="Testing")
public class Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Jerry.init(hardwareMap);
        Jerry.movementPower = 0.6;
        Localizer localizer = new Localizer(this, hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // Movement
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Jerry.movementPower;
            Jerry.drivetrain.drive(magnitude, theta, driveTurn, movementPower);
            localizer.update();

            telemetry.addData("X: ", localizer.getX());
            telemetry.addData("Y: ", localizer.getY());
            telemetry.addData("Heading: ", localizer.getHeading());
            telemetry.addData("RawX: ", localizer.getRawX());
            telemetry.addData("RawY: ", localizer.getRawY());
            telemetry.update();


            // Outtake Slides



        }


    }
}
