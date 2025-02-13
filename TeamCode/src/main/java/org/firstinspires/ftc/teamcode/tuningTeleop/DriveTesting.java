package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp(name="Testing")
public class DriveTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
//        Localizer localizer = new Localizer(this, hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // Movement
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
            drive.drive(magnitude, theta, driveTurn, 0.7);

//            telemetry.addData("X: ", Pika.localizer.getX());
//            telemetry.addData("Y: ", Pika.localizer.getY());
//            telemetry.addData("Heading: ", Pika.localizer.getHeading(Localizer.Angle.DEGREES));
//            telemetry.addData("RawX: ", Pika.localizer.getRawX());
//            telemetry.addData("RawY: ", Pika.localizer.getRawY());
//            telemetry.addData("RightEnc: ", Pika.localizer.getRightEncoderPosition());
//            telemetry.addData("LeftEnc: ", Pika.localizer.getLeftEncoderPosition());
//            telemetry.addData("PerpEnc: ", Pika.localizer.getFwdEncoderPosition());
//            telemetry.addData("DriveTurn: ", driveTurn);
//            telemetry.addData("Timer: ", timer.milliseconds());
//            telemetry.addData("", Pika.drivetrain.getTelemetry());
            telemetry.update();
            timer.reset();


            // Outtake Slides



        }


    }
}
