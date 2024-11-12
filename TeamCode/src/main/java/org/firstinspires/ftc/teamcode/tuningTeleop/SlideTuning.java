package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class SlideTuning extends LinearOpMode {
    boolean up = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this);
        waitForStart();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );

        while (opModeIsActive()) {
            if (xReader.wasJustReleased()) {
                if (!up)
                    Pika.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.INTAKE.getTicks());
                else
                    Pika.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.BUCKET2.getTicks());
                up = !up;
            }

            if (gamepad1.dpad_up) {
                Pika.outtakeSlides.goUp();
            }
            else if (gamepad1.dpad_down) {
                Pika.outtakeSlides.goDown();
            }
            else {
                Pika.outtakeSlides.stopSlides();
            }
            xReader.readValue();
//            Jerry.outtakeSlides.update();
            telemetry.addData("Target Position: ", Pika.outtakeSlides.getTargetPosition());
            telemetry.addData("Current Pos: ", Pika.outtakeSlides.getCurrentPosition());
            telemetry.addData("Power: ", Pika.outtakeSlides.getPW());
            telemetry.update();
        }
    }
}
