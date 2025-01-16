package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
@Config
public class FullArmTest extends LinearOpMode {
    boolean up = false;
    public static double P = 0, I = 0, D = 0;
    public static int slidePos = 0;
    boolean pitching = true;

    public static int armPos = 0;
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();

        while (opModeIsActive()) {
            pitching = false;
            if (gamepad1.dpad_right && armPos <=1500) {
                armPos +=2;
                pitching = true;
            }
            else if (gamepad1.dpad_left && armPos >=0) {
                armPos -=2;
                pitching = true;
            }

            if (gamepad1.right_bumper) {
                slidePos +=25;
            }
            else if (gamepad1.left_bumper) {
                slidePos -=25;
            }

            if (pitching) {
                Pika.outtakeSlides.freeMove();
                slidePos = Pika.outtakeSlides.getCurrentPosition();
            }
            else {
                Pika.outtakeSlides.update();
            }

            Pika.outtakeSlides.setTargetPosition(slidePos);
            Pika.arm.setTargetPosition(armPos);
            Pika.arm.update();

            telemetry.addData("Pitching: ", pitching);
            telemetry.addData("", Pika.arm.getTelemetry());
            telemetry.addData("\nSlides: ", "");

            telemetry.addData("", Pika.outtakeSlides.getTelemetry());
            telemetry.update();
        }
    }
}
