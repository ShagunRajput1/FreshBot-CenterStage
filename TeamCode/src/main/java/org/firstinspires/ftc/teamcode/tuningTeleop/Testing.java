package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp(name="Testing")
public class Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this);
        Pika.movementPower = 0.55;
//        Localizer localizer = new Localizer(this, hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // Movement
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 1);
            double driveX = Math.pow(-gamepad2.left_stick_y, 1);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Pika.movementPower;
            Pika.drivetrain.drive(magnitude, theta, driveTurn, movementPower);
            Pika.localizer.update();

            telemetry.addData("X: ", Pika.localizer.getX());
            telemetry.addData("Y: ", Pika.localizer.getY());
            telemetry.addData("Heading: ", Pika.localizer.getHeading(Localizer.Angle.DEGREES));
            telemetry.addData("RawX: ", Pika.localizer.getRawX());
            telemetry.addData("RawY: ", Pika.localizer.getRawY());
            telemetry.addData("RightEnc: ", Pika.localizer.getRightEncoderPosition());
            telemetry.addData("LeftEnc: ", Pika.localizer.getLeftEncoderPosition());
            telemetry.addData("PerpEnc: ", Pika.localizer.getFwdEncoderPosition());
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.addData("", Pika.drivetrain.getTelemetry());
            telemetry.update();


            // Outtake Slides



        }


    }
}
