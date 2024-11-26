package org.firstinspires.ftc.teamcode.tuningTeleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.Camlight;

@TeleOp
public class LimelightTest extends LinearOpMode {

    private Camlight limelight;
    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = new Camlight();
        limelight.init(hardwareMap);
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        waitForStart();
        while (opModeIsActive()) {
            String corners = "";
            double angle = limelight.getSampleOrientation();
            if (angle == -1) {
                telemetry.addData("No Sample Detected", "");
            }
            else {
                telemetry.addData("Angle: ", angle);
                telemetry.addData("Delta Y: ", limelight.deltaY);
                telemetry.addData("Delta X: ", limelight.deltaX);
                if (limelight.cornerIndices != null) {
                    corners= corners.concat("[");
                    for (int i = 0; i < 4; i++) {
                        corners = corners.concat( limelight.cornerIndices[i] + ", ");
                    }
                    corners = corners.concat("]\n\n");
                }
                if (limelight.corners != null) {
                    for (int i = 0; i < limelight.corners.size(); i++) {
                        corners = corners.concat("[" + limelight.corners.get(i).get(0) + " " + limelight.corners.get(i).get(1) + "]\n");
                    }
                }
                telemetry.addData("Corners In Order: ", corners);
                telemetry.addData("TiltLeft: ", limelight.tiltedLeft);
            }
            telemetry.update();
        }
    }
}
