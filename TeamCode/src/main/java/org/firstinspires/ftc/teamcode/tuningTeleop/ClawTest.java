package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.util.RunningStats;

@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        RunningStats angleAverages = new RunningStats(500, 2.5);
        Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
        Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition());
        Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.GRAB.getPosition());
        double orientation = 0;
        double prevOrientation = 0;
        waitForStart();

        while (opModeIsActive()) {
//            Jerry.claw.setClaw(0);




            if (gamepad1.x) {
                Pika.newClaw.setPivot(Pika.newClaw.pivotPos+=0.001);
            }
            else if (gamepad1.b) {
                Pika.newClaw.setPivot(Pika.newClaw.pivotPos-=0.001);
            }
            orientation = Pika.limelight.getSampleOrientation();
            double diff = Math.abs(orientation-prevOrientation);
            if (orientation != -1 && (diff>=10 && diff<=140)) {
                prevOrientation = orientation;
//                angleAverages.addDataPoint(orientation);
                Pika.newClaw.setPivotOrientation(orientation);
            }

            telemetry.addData("ClawPos: ", Pika.newClaw.clawPos);
            telemetry.addData("PivotPos: ", Pika.newClaw.pivotPos);
            telemetry.addData("Orientation: ", orientation);
            telemetry.addData("Mean Angle: ", angleAverages.getMean());
            telemetry.addData("StdDev: ", angleAverages.getStandardDeviation());
            telemetry.addData("Q: ", angleAverages.printQ());
            telemetry.update();
        }
    }
}
