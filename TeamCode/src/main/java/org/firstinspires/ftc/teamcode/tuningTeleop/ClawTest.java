package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.util.RunningStats;

@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this);
        RunningStats angleAverages = new RunningStats(500, 2.5);
        waitForStart();

        while (opModeIsActive()) {
//            Jerry.claw.setClaw(0);

            if (gamepad1.a) {
                Pika.claw.setClaw(Pika.claw.clawPos+=0.01);
            }
            else if (gamepad1.y) {
                Pika.claw.setClaw(Pika.claw.clawPos-=0.01);
            }

            if (gamepad1.x) {
                Pika.claw.setPivot(Pika.claw.pivotPos+=0.001);
            }
            else if (gamepad1.b) {
                Pika.claw.setPivot(Pika.claw.pivotPos-=0.001);
            }
            double orientation = Pika.limelight.getSampleOrientationNN();
            if (orientation != -1) {
//                angleAverages.addDataPoint(orientation);
                Pika.claw.setPivotOrientation(orientation);
            }
            telemetry.addData("ClawPos: ", Pika.claw.clawPos);
            telemetry.addData("PivotPos: ", Pika.claw.pivotPos);
            telemetry.addData("Orientation: ", orientation);
            telemetry.addData("Mean Angle: ", angleAverages.getMean());
            telemetry.addData("StdDev: ", angleAverages.getStandardDeviation());
            telemetry.addData("Q: ", angleAverages.printQ());
            telemetry.update();
        }
    }
}
