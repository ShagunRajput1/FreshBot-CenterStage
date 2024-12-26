package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.AutoIntakeFromSub;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp
public class AutoIntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();
        ToggleButtonReader xReader = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.X);
        AutoIntakeFromSub autoIntake = new AutoIntakeFromSub(new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap));
        while (opModeIsActive()) {
            Pika.localizer.update();
            if (xReader.wasJustReleased()) {
                autoIntake.init();
            }
            if (autoIntake.isFinished() && Pika.arm.isFinished()) {
                Pika.outtakeSlides.update();
            }
            Pika.arm.update();
            autoIntake.update();
            xReader.readValue();
            telemetry.addData("Slides: ", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("", Pika.limelight.getTelemetry());
            telemetry.addData("IntakeDone: ", autoIntake.isFinished());
            telemetry.update();
        }
    }
}
