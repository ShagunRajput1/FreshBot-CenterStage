package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleSpec;
import org.firstinspires.ftc.teamcode.commandBase.PrepareSpecDeposit;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class SpecCycleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        ToggleButtonReader intakeSpec = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_LEFT);
        ToggleButtonReader outtakeSpec = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER);
        ToggleButtonReader clawReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        IntakeSampleSpec intakeSampleSpec = new IntakeSampleSpec();
        PrepareSpecDeposit prepareSpecDeposit = new PrepareSpecDeposit();

        waitForStart();
        while (opModeIsActive()) {
            if (intakeSpec.wasJustReleased()) {
                intakeSampleSpec.init();
                prepareSpecDeposit.stop();
            }

            if (outtakeSpec.wasJustReleased()) {
                prepareSpecDeposit.init();
                intakeSampleSpec.stop();
            }
            if (clawReader.wasJustReleased()) {
                if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.OPEN.getPosition()) {
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition());
                }
                else {
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                }
            }


            if (gamepad1.dpad_up) {
                Pika.outtakeSlides.goUp();
            }
            else if (gamepad1.dpad_down) {
                Pika.outtakeSlides.goDown();
            }
            else {
                Pika.outtakeSlides.freeMove();
            }

            Pika.arm.update();
            intakeSampleSpec.update();
            prepareSpecDeposit.update();
            clawReader.readValue();
            intakeSpec.readValue();
            outtakeSpec.readValue();
            Pika.localizer.update();
            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
            telemetry.addData("Slides: ", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("End Arm: ", Pika.newClaw.getTelemetry());
            telemetry.addData("Localizer: ", Pika.localizer.getTelemetry());
            telemetry.update();
        }
    }
}
