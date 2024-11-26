package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;

@Config
@TeleOp
public class ArmTest extends LinearOpMode {
    public static double P = 0.007, I = 0.00025, D = 0;
    public static int targetPos = 0;

    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.right_bumper && targetPos<=2000) {
                targetPos+=2;
            }
            else if (gamepad1.left_bumper && targetPos>=0) {
                targetPos-=2;
            }
            Pika.arm.setPID(P, I, D);
            Pika.arm.setTargetPosition(targetPos);
            Pika.arm.update();

            telemetry.addData("", Pika.arm.getTelemetry());
            telemetry.update();
        }
    }

}
