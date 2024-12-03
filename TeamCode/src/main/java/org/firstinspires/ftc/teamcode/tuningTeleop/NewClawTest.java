package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class NewClawTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        waitForStart();
        double pos = FinalClaw.ArmPitch.RETRACT.getPosition();
        double miniPitchPos = FinalClaw.MiniPitch.GRAB.getPosition();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pos += 0.0005;
            }
            else if (gamepad1.dpad_down) {
                pos -=0.0005;
            }

            if (gamepad1.a) {
                miniPitchPos += 0.0005;
            }
            else if (gamepad1.y) {
                miniPitchPos -= 0.0005;
            }

            Pika.newClaw.pitchA.setPosition(pos);
            Pika.newClaw.pitchB.setPosition(1-pos);
            Pika.newClaw.miniPitch.setPosition(miniPitchPos);


            telemetry.addData("PitchAPos: ", pos);
            telemetry.addData("PitchBPos: ", (1-pos));
            telemetry.addData("MiniPitchPos: ", miniPitchPos);
            telemetry.update();
        }
    }
}
