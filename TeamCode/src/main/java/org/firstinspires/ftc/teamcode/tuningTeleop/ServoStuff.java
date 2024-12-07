package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoStuff extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1;
        Servo s2;
        s1 = hardwareMap.get(Servo.class, "s1");
        double pos = 0;
        s1.setPosition(pos);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper && pos<1) {
                pos+=0.0005;
            }
            else if (gamepad1.left_bumper && pos>0) {
                pos-=0.0005;
            }
            s1.setPosition(pos);

            telemetry.addData("Pos: ", pos);
            telemetry.update();
        }
    }
}
