package org.firstinspires.ftc.teamcode.tuningTeleop;

import android.media.JetPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class SlideTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("", Pika.arm.getTelemetry());
            telemetry.update();
        }
    }

}
