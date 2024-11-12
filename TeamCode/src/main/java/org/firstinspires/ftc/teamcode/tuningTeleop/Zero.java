package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.Provider;

@TeleOp
public class Zero extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "s1");
        waitForStart();

        while(opModeIsActive()) {
            s1.setPosition(0);
        }
    }
}
