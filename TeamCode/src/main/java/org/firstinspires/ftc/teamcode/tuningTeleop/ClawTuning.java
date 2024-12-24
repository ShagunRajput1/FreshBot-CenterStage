package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class ClawTuning extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        double clawPos = FinalClaw.ClawPosition.OPEN.getPosition();
        Pika.init(hardwareMap, this, false);
        Pika.newClaw.setClaw(clawPos);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && clawPos<1) {
                clawPos += 0.0005;
            }
            else if (gamepad1.b && clawPos>0) {
                clawPos -= 0.0005;
            }
            Pika.newClaw.setClaw(clawPos);

            telemetry.addData("ClawPos: ", clawPos);
            telemetry.update();

        }
    }
}
