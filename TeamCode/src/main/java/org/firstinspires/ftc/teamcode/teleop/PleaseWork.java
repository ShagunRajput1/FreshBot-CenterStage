package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Outtake;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class PleaseWork extends LinearOpMode {
    boolean pivotDown;
    boolean bucketDown;
    public void runOpMode() throws InterruptedException {
        pivotDown = false;
        bucketDown = true;
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );


        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        ToggleButtonReader bReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );

        Pika.init(hardwareMap, this);
        Pika.movementPower = 0.6;

        waitForStart();
        while (opModeIsActive()) {
            // Movement
            double driveTurn = -Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Pika.movementPower;
            Pika.drivetrain.drive(magnitude, theta, driveTurn, movementPower);

            // Outtake and Intake stuff
            if (gamepad1.dpad_up) {
                Pika.outtakeSlides.goUp();
            }
            else if (gamepad1.dpad_down) {
                Pika.outtakeSlides.goDown();
            }
            else {
                Pika.outtakeSlides.stopSlides();
            }

            if (gamepad1.right_bumper) {
                Pika.intake.setPivotPosition(Intake.PivotPosition.Down.getPosition());
                Pika.intake.extend();
            }
            else if (gamepad1.left_bumper) {
                Pika.intake.retract();
            }

            if (xReader.wasJustReleased()) {
                if (pivotDown)
                    Pika.intake.setPivotPosition(Intake.PivotPosition.Up.getPosition());
                else
                    Pika.intake.setPivotPosition(Intake.PivotPosition.Down.getPosition());
                pivotDown = !pivotDown;
            }

            if (bReader.wasJustReleased()) {
                Pika.intake.setPivotPosition(Intake.PivotPosition.Up.getPosition());
                Pika.intake.setExtendoPostion(Intake.ExtensionPosition.RETRACTED.getPosition());
            }

            if (gamepad1.y) {
                Pika.outtake.liftArm();
            }

            if (aReader.wasJustReleased()) {
                    Pika.outtake.setArmPosition(Outtake.ArmPos.LOAD.getPos());
            }
            if (gamepad1.dpad_left) {
                Pika.intake.rollerIn();
            }
            else if(gamepad1.dpad_right) {
                Pika.intake.rollerOut();
            }
            else {
                Pika.intake.rollerStop();
            }


            aReader.readValue();
            xReader.readValue();
            bReader.readValue();

            telemetry.update();


            // Outtake Slides



        }

    }
}
