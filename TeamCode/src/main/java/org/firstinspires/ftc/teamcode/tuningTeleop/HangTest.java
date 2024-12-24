package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class HangTest extends LinearOpMode {
    int armPos, slidePos;
    boolean slidePositionMode = false;
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader yReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                armPos = Arm.ArmPos.PREP_HANG.getPosition();
            }
            else if (gamepad1.dpad_down) {
                slidePositionMode = true;
                armPos = Arm.ArmPos.HANG.getPosition();
                Pika.outtakeSlides.setTargetPosition(
                        OuttakeSlides.TurnValue.HANG_RETRACT.getTicks()
                );
            }

            if (yReader.wasJustReleased()) {
                armPos = Arm.ArmPos.STUPID_TAIL.getPosition();
            }

            if (gamepad1.right_bumper) {
                slidePositionMode = false;
                Pika.outtakeSlides.setPower(0.4);
                Pika.outtakeSlides.goUp();
            }
            else if (gamepad1.left_bumper) {
                Pika.outtakeSlides.setPower(0.4);
                slidePositionMode = false;
                Pika.outtakeSlides.goDown();
            }
            else {
                slidePositionMode = true;
                Pika.outtakeSlides.holdSlides();
            }
            Pika.arm.setTargetPosition(armPos);
            if (slidePositionMode && Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            yReader.readValue();
            Pika.arm.update();
            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
            telemetry.addData("ArmPos: ", armPos);
            telemetry.addData("Slides: ", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("SlidePositionMode: ", slidePositionMode);
            telemetry.update();

        }
    }
}
