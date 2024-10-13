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
            telemetry.addData("Angle: ", limelight.getSampleOrientation());
            telemetry.addData("Y: ", limelight.deltaY);
            telemetry.addData("X: ", limelight.deltaX);
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
            telemetry.addData("Corners: ", corners);
            telemetry.update();
        }
    }
}
