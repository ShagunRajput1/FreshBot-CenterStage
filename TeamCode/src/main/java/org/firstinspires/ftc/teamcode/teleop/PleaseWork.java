package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Outtake;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Jerry;

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

        Jerry.init(hardwareMap);
        Jerry.movementPower = 0.6;

        waitForStart();
        while (opModeIsActive()) {
            // Movement
            double driveTurn = -Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Jerry.movementPower;
            Jerry.drivetrain.drive(magnitude, theta, driveTurn, movementPower);

            // Outtake and Intake stuff
            if (gamepad1.dpad_up) {
                Jerry.outtakeSlides.goUp();
            }
            else if (gamepad1.dpad_down) {
                Jerry.outtakeSlides.goDown();
            }
            else {
                Jerry.outtakeSlides.stopSlides();
            }

            if (gamepad1.right_bumper) {
                Jerry.intake.setPivotPosition(Intake.PivotPosition.Down.getPosition());
                Jerry.intake.extend();
            }
            else if (gamepad1.left_bumper) {
                Jerry.intake.retract();
            }

            if (xReader.wasJustReleased()) {
                if (pivotDown)
                    Jerry.intake.setPivotPosition(Intake.PivotPosition.Up.getPosition());
                else
                    Jerry.intake.setPivotPosition(Intake.PivotPosition.Down.getPosition());
                pivotDown = !pivotDown;
            }

            if (bReader.wasJustReleased()) {
                Jerry.intake.setPivotPosition(Intake.PivotPosition.Up.getPosition());
                Jerry.intake.setExtendoPostion(Intake.ExtensionPosition.RETRACTED.getPosition());
            }

            if (gamepad1.y) {
                Jerry.outtake.liftArm();
            }

            if (aReader.wasJustReleased()) {
                    Jerry.outtake.setArmPosition(Outtake.ArmPos.LOAD.getPos());
            }
            if (gamepad1.dpad_left) {
                Jerry.intake.rollerIn();
            }
            else if(gamepad1.dpad_right) {
                Jerry.intake.rollerOut();
            }
            else {
                Jerry.intake.rollerStop();
            }


            aReader.readValue();
            xReader.readValue();
            bReader.readValue();

            telemetry.update();


            // Outtake Slides



        }

    }
}
