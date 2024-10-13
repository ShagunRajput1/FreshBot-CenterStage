package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Jerry;

@TeleOp
public class FindIntakePositions extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Jerry.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                Jerry.intake.extend();
            }
            else if (gamepad1.left_bumper) {
                Jerry.intake.retract();
            }
            if (gamepad1.a) {
                Jerry.intake.incrementPivot();
            }
            else if (gamepad1.b) {
                Jerry.intake.decrementPivot();
            }
            if (gamepad1.dpad_left) {
                Jerry.intake.rollerIn();
            }
            else if (gamepad1.dpad_right) {
                Jerry.intake.rollerOut();
            }
            else {
                Jerry.intake.rollerStop();
            }

            if (gamepad1.dpad_up) {
                Jerry.outtake.liftArm();
            }
            else if (gamepad1.dpad_down) {
                Jerry.outtake.lowerArm();
            }
            Jerry.outtakeSlides.holdSlide();
            telemetry.addData("Current Pos: ", Jerry.outtakeSlides.getCurrentPosition());
            telemetry.addData("Power: ", Jerry.outtakeSlides.getPW());


            telemetry.addData("Extendo pos: ", Jerry.intake.getPosition());
            telemetry.addData("ArmPos: ", Jerry.outtake.getArmPosition());
            telemetry.addData("Dist sensor: ", Jerry.intake.getDistance());
            telemetry.update();
        }
    }

}
