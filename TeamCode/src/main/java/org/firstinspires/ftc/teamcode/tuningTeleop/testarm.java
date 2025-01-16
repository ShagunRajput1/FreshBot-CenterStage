package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.core.Pika;

public class testarm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        ToggleButtonReader xReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        IntakeSampleTeleOp intake = new IntakeSampleTeleOp();
        waitForStart();

        while (opModeIsActive()) {
            if (xReader.wasJustReleased()) {
                intake.init();
            }
            xReader.readValue();
            intake.update();
        }
    }
}
