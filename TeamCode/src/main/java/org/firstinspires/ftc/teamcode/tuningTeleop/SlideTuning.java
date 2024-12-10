package org.firstinspires.ftc.teamcode.tuningTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

@Config
@TeleOp
public class SlideTuning extends LinearOpMode {
    boolean up = false;
    public static double P = 0.000275, I = 0.000008, D = 0;
    public static int pos = 0;
    public static double holdConstant = 0;
    boolean positionMode = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        int holdPos = 0;
        Pika.arm.setTargetPosition(Arm.ArmPos.OUTTAKE.getPosition());
        waitForStart();

        while (opModeIsActive()) {
            Pika.outtakeSlides.setTargetPosition(pos);
            if (gamepad1.right_bumper) {
                positionMode = true;
                Pika.outtakeSlides.setTargetPosition(Pika.outtakeSlides.targetPos+150);
            }
            else if (gamepad1.left_bumper && Pika.outtakeSlides.targetPos>=0) {
                positionMode = true;
                Pika.outtakeSlides.setTargetPosition(Pika.outtakeSlides.targetPos-150);
            }

            if (gamepad1.dpad_up) {
                positionMode = false;
                Pika.outtakeSlides.goUp();
                holdPos = Pika.outtakeSlides.getCurrentPosition();
            }
            else if (gamepad1.dpad_down) {
                positionMode = false;
                Pika.outtakeSlides.goDown();
                holdPos = Pika.outtakeSlides.getCurrentPosition();
            }
            else {
                positionMode = true;
//                Pika.outtakeSlides.setTargetPosition(holdPos);
            }

//            Pika.outtakeSlides.setHoldConstant(holdConstant);


            if (positionMode && Pika.arm.isFinished()) {
                Pika.outtakeSlides.setPID(P, I, D);
                Pika.outtakeSlides.update();
            }
            Pika.arm.update();
//            else {
//                Pika.outtakeSlides.holdSlides();
//            }
            telemetry.addData("PositionMode: ", positionMode);
            telemetry.addData("", Pika.outtakeSlides.getTelemetry());
            telemetry.update();
        }
    }
}
