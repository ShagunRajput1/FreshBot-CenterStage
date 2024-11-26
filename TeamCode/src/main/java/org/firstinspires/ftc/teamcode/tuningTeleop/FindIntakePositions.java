package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class FindIntakePositions extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                Pika.intake.extend();
            }
            else if (gamepad1.left_bumper) {
                Pika.intake.retract();
            }
            if (gamepad1.a) {
                Pika.intake.incrementPivot();
            }
            else if (gamepad1.b) {
                Pika.intake.decrementPivot();
            }
            if (gamepad1.dpad_left) {
                Pika.intake.rollerIn();
            }
            else if (gamepad1.dpad_right) {
                Pika.intake.rollerOut();
            }
            else {
                Pika.intake.rollerStop();
            }

            if (gamepad1.dpad_up) {
                Pika.outtake.liftArm();
            }
            else if (gamepad1.dpad_down) {
                Pika.outtake.lowerArm();
            }
            telemetry.addData("Current Pos: ", Pika.outtakeSlides.getCurrentPosition());
            telemetry.addData("Power: ", Pika.outtakeSlides.getPW());


            telemetry.addData("Extendo pos: ", Pika.intake.getPosition());
            telemetry.addData("ArmPos: ", Pika.outtake.getArmPosition());
            telemetry.addData("Dist sensor: ", Pika.intake.getDistance());
            telemetry.update();
        }
    }

}
