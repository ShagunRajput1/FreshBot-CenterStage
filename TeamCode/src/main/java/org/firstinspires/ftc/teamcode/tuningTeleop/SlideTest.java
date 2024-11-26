package org.firstinspires.ftc.teamcode.tuningTeleop;

import android.media.JetPlayer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.Pika;

@Config
@TeleOp
public class SlideTest extends LinearOpMode {
    DcMotorEx slide1;
    DcMotorEx slide2;
    public static double pw = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                slide1.setPower(pw);
                slide2.setPower(-pw);
            }
            else if (gamepad1.left_bumper) {
                slide1.setPower(-pw);
                slide2.setPower(pw);
            }
            else {
                slide1.setPower(0);
                slide2.setPower(0);
            }
            telemetry.addData("SlidePos: ", slide1.getCurrentPosition());
            telemetry.update();
        }


    }

}
