package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Jerry;

@TeleOp
public class SlideTuning extends LinearOpMode {
    boolean up = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Jerry.init(hardwareMap);
        waitForStart();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );

        while (opModeIsActive()) {
            if (xReader.wasJustReleased()) {
                if (!up)
                    Jerry.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.INTAKE.getTicks());
                else
                    Jerry.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.BUCKET2.getTicks());
                up = !up;
            }

            if (gamepad1.dpad_up) {
                Jerry.outtakeSlides.goUp();
            }
            else if (gamepad1.dpad_down) {
                Jerry.outtakeSlides.goDown();
            }
            else {
                Jerry.outtakeSlides.stopSlides();
            }
            xReader.readValue();
//            Jerry.outtakeSlides.update();
            telemetry.addData("Target Position: ", Jerry.outtakeSlides.getTargetPosition());
            telemetry.addData("Current Pos: ", Jerry.outtakeSlides.getCurrentPosition());
            telemetry.addData("Power: ", Jerry.outtakeSlides.getPW());
            telemetry.update();
        }
    }
}
