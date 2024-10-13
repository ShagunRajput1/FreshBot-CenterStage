package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Jerry;

@TeleOp
public class FindOuttakePos extends LinearOpMode {
    DcMotorEx slide1;
    DcMotorEx slide2;
    @Override
    public void runOpMode() throws InterruptedException {
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                slide1.setPower(0.3);
                slide2.setPower(0.3);
            }
            else if (gamepad1.left_bumper) {
                slide1.setPower(-0.3);
                slide2.setPower(-0.3);
            }
            else {
                slide1.setPower(0);
                slide2.setPower(0);
            }

        }
    }
}
