package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp
@Config
public class SohamsRobot extends LinearOpMode {
    public static double armPowerUp = 0.5;
    public static double armPowerDown = 0.25;
    public static double P, I, D;
    public static double extendoPower = 0.5;
    int armHoldPos = 0;
    boolean armPosMode = true;
    double openClawPos = 0.646;
    double closeClawPos = 1;
    boolean clawOpen;

    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new MecanumDrive(hardwareMap);
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm1");
        CRServo extendo = hardwareMap.get(CRServo.class, "extendo");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        clawOpen = true;
        claw.setPosition(openClawPos);
        PIDController armController = new PIDController(0.002, 0.0002, 0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setTargetPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            double driveTurn = Math.pow(-gamepad1.right_stick_x, 1);
            double driveY = -Math.pow(-gamepad1.left_stick_x, 1);
            double driveX = -Math.pow(-gamepad1.left_stick_y, 1);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            drivetrain.drive(magnitude, theta, driveTurn, 0.4);
            if (gamepad1.dpad_up && armMotor.getCurrentPosition()<760) {
                armMotor.setPower(armPowerUp);
                armPosMode = false;
            }
            else if (gamepad1.dpad_down && armMotor.getCurrentPosition()>0) {
                armMotor.setPower(-armPowerDown);
                armPosMode = false;
            }
            else {
                armPosMode = true;
            }

            if (gamepad1.right_bumper) {
                extendo.setPower(extendoPower);
            }
            else if (gamepad1.left_bumper) {
                extendo.setPower(-extendoPower);
            }
            else {
                extendo.setPower(0);
            }
            if (!armPosMode) {
                armHoldPos = armMotor.getCurrentPosition();
            }
            else {
                double error = armHoldPos - armMotor.getCurrentPosition();
                error = (Math.abs(error)>50) ? error: 0;
                double power = Range.clip(armController.calculate(0, error), -1, 1);
                armMotor.setPower(power);
            }

            if (xReader.wasJustReleased()) {
                if (clawOpen) {
                    claw.setPosition(closeClawPos);
                } else {
                    claw.setPosition(openClawPos);
                }
                clawOpen = !clawOpen;
            }
            xReader.readValue();
            telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
            telemetry.update();


        }
    }
}

/*
lf: 1
rf: 3
lb: 0
rb: 2

* */
