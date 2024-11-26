package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.util.RunningStats;

@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        RunningStats angleAverages = new RunningStats(500, 2.5);
        Servo clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        double clawPitchPos = 0;
        double orientation = 0;
        double prevOrientation = 0;
        clawPitch.setPosition(clawPitchPos);
        waitForStart();

        while (opModeIsActive()) {
//            Jerry.claw.setClaw(0);

            if (gamepad1.a) {
                Pika.claw.setClaw(Pika.claw.clawPos+=0.01);
            }
            else if (gamepad1.y) {
                Pika.claw.setClaw(Pika.claw.clawPos-=0.01);
            }

            if (gamepad1.dpad_left && clawPitchPos<=1) {
                clawPitchPos+=0.0005;
            }
            else if (gamepad1.dpad_right && clawPitchPos>=0) {
                clawPitchPos-=0.0005;
            }

            if (gamepad1.x) {
                Pika.claw.setPivot(Pika.claw.pivotPos+=0.001);
            }
            else if (gamepad1.b) {
                Pika.claw.setPivot(Pika.claw.pivotPos-=0.001);
            }
            orientation = Pika.limelight.getSampleOrientation();
            double diff = Math.abs(orientation-prevOrientation);
            if (orientation != -1 && (diff>=10 && diff<=140)) {
                prevOrientation = orientation;
//                angleAverages.addDataPoint(orientation);
                Pika.claw.setPivotOrientation(orientation);
            }
            clawPitch.setPosition(clawPitchPos);

            telemetry.addData("ClawPos: ", Pika.claw.clawPos);
            telemetry.addData("ClawPitchPos: ", clawPitchPos);
            telemetry.addData("PivotPos: ", Pika.claw.pivotPos);
            telemetry.addData("Orientation: ", orientation);
            telemetry.addData("Mean Angle: ", angleAverages.getMean());
            telemetry.addData("StdDev: ", angleAverages.getStandardDeviation());
            telemetry.addData("Q: ", angleAverages.printQ());
            telemetry.update();
        }
    }
}
