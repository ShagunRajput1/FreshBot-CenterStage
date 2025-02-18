package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp
public class ImuTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);

        waitForStart();

        while (opModeIsActive()) {
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 1);

            driveTurn = (Math.abs(driveTurn) > 0) ?
                    (driveTurn + Math.signum(driveTurn)* MotionPlannerEdit.kStatic_Turn) : 0;
            double driveY = Math.pow(-gamepad2.left_stick_x, 1);
            driveY = (Math.abs(driveY) > 0) ?
                    driveY + Math.signum(driveY)*MotionPlannerEdit.kStatic_Y : 0;
            double driveX = Math.pow(-gamepad2.left_stick_y, 1);
            driveX = (Math.abs(driveX) > 0) ?
                    driveX + Math.signum(driveX)*MotionPlannerEdit.kStatic_X : 0;

            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Pika.movementPower;
            Pika.drivetrain.drive(magnitude, theta, driveTurn, 0.7);

            Pika.localizer.update();
            telemetry.addData("Heading: ", Pika.localizer.getHeadingImu());
            telemetry.update();
        }
    }
}
