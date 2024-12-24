package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class NewClawTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        boolean clawOpen = true;
        double pos = FinalClaw.ArmPitch.RETRACT.getPosition();
        double miniPitchPos = FinalClaw.MiniPitch.RETRACT.getPosition();
        double clawPos = FinalClaw.ClawPosition.CLOSE.getPosition();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        waitForStart();

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


           if (xReader.wasJustReleased()) {
               clawOpen = !clawOpen;
               if (clawOpen) {
                   Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition());
               }
               else
                   Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
           }


            if (gamepad1.right_trigger>0 && Pika.newClaw.orientation<=180) {
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation+(8*gamepad1.right_trigger));
            }
            else if (gamepad1.left_trigger>0 && Pika.newClaw.orientation>=0) {
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation-(8*gamepad1.left_trigger));
            }


            Pika.newClaw.pitchA.setPosition(pos);
            Pika.newClaw.pitchB.setPosition(1-pos);
            Pika.newClaw.miniPitch.setPosition(miniPitchPos);

            xReader.readValue();
            telemetry.addData("PitchAPos: ", pos);
            telemetry.addData("PitchBPos: ", (1-pos));
            telemetry.addData("MiniPitchPos: ", miniPitchPos);
            telemetry.addData("claw: ", clawPos);
            telemetry.update();
        }
    }
}
