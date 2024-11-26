package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Pika;

@TeleOp
public class DifferentialClawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        double pivotPos = 0.5;
        double pivotAngle = 90;
        double clawPos = 0;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper && pivotAngle<=180) {
                pivotAngle+=0.05;
                pivotPos+=0.0005;
            }
            else if (gamepad1.left_bumper) {
                pivotAngle-=0.05;
                pivotPos-=0.0005;

            }

            if (gamepad1.dpad_up) {
                Pika.claw.upPitchPosition(0.0005);
            }
            else if (gamepad1.dpad_down) {
                Pika.claw.downPitchPosition(0.0005);
            }


            if (gamepad1.a && clawPos<=1) {
                clawPos+=0.0005;
            }
            else if (gamepad1.b && clawPos>=0) {
                clawPos-=0.0005;
            }
            Pika.claw.setClaw(clawPos);
            Pika.claw.setPivotOrientation(pivotAngle);

            telemetry.addData("A Position: ", Pika.claw.pivotPitchA.getPosition());
            telemetry.addData("B Position: ", Pika.claw.pivotPitchB.getPosition());
            telemetry.addData("PivotAngle: ", pivotAngle);
            telemetry.addData("ClawPos: ", clawPos);
            telemetry.update();

        }
    }
}
