package org.firstinspires.ftc.teamcode.tuningTeleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class ProximityTest extends LinearOpMode {
    DistanceSensor distanceSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Dist: ", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
