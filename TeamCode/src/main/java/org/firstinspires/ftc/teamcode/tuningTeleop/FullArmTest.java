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
    int pos = 0;
    boolean pitching = true;

    int targetPos = 0;
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();

        while (opModeIsActive()) {
            pitching = false;
            if (gamepad1.dpad_right && targetPos<=1500) {
                targetPos+=2;
                pitching = true;
            }
            else if (gamepad1.dpad_left && targetPos>=0) {
                targetPos-=2;
                pitching = true;
            }

            if (gamepad1.right_bumper) {
                pos+=25;
            }
            else if (gamepad1.left_bumper) {
                pos-=25;
            }

            if (pitching) {
                Pika.outtakeSlides.freeMove();
                pos = Pika.outtakeSlides.getCurrentPosition();
            }
            else {
                Pika.outtakeSlides.update();
            }

            Pika.outtakeSlides.setTargetPosition(pos);
            Pika.arm.setTargetPosition(targetPos);
            Pika.arm.update();

            telemetry.addData("Pitching: ", pitching);
            telemetry.addData("", Pika.arm.getTelemetry());
            telemetry.addData("\nSlides: ", "");

            telemetry.addData("", Pika.outtakeSlides.getTelemetry());
            telemetry.update();
        }
    }
}
