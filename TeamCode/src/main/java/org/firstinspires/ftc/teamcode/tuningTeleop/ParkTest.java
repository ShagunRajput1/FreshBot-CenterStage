package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.PreparePark;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class ParkTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        ToggleButtonReader xReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        PreparePark preparePark = new PreparePark();
        waitForStart();
        while (opModeIsActive()) {
            if (xReader.wasJustReleased()) {
                preparePark.init();
            }
            preparePark.update();
            xReader.readValue();
        }
    }
}
