package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class NewIntakeTest extends LinearOpMode {
    CRServo s1;
    CRServo s2;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        waitForStart();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader bReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );
        int mult = -1;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                if (distanceSensor.getDistance(DistanceUnit.MM) > 10 ) {
                    s1.setPower(mult);
                    s2.setPower(-mult);
                }
                else {
                    s1.setPower(0);
                    s1.setPower(0);
                }
            }
            else if (gamepad1.right_bumper) {
                s1.setPower(-mult);
                s2.setPower(mult);
            }
            else {
                s1.setPower(0);
                s2.setPower(0);
            }
        }
    }
}
