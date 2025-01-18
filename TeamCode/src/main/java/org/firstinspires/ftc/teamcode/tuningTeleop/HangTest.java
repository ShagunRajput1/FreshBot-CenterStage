package org.firstinspires.ftc.teamcode.tuningTeleop;

import android.database.AbstractCursor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.ActualTeleOpHang;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class HangTest extends LinearOpMode {
    int armPos, slidePos;
    boolean slidePositionMode = false;
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader hangButton = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        ActualTeleOpHang hang = new ActualTeleOpHang();
        waitForStart();
        while (opModeIsActive()) {

            if (hangButton.wasJustReleased()) {
                if (Pika.arm.getTargetPosition() != Arm.ArmPos.PREP_HANG.getPosition()) {
                    slidePositionMode = true;
                    hang.init();
                }
            }

            if (slidePositionMode && Pika.arm.isFinished())
                Pika.outtakeSlides.update();
            hangButton.readValue();
            hang.update();
            Pika.arm.update();
            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
            telemetry.addData("ArmPos: ", armPos);
            telemetry.addData("Slides: ", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("SlidePositionMode: ", slidePositionMode);
            telemetry.update();

        }
    }
}
