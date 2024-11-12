package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TrunkOrTreatArm extends LinearOpMode {
    double clawPos, shoulderPos, pivotPos, elbowPos, turretPos;
    final double shoulderMaxPos = 0.78, elbowMaxPos = 0.3894, pivotMaxPos=1;
    double shoulderAngle = 90, elbowAngle = 0, pivotAngle;
    double extendMagnitude, turnMagnitude;

    CRServo claw;
    Servo shoulder;
    Servo elbow;
    CRServo pivot;
    Servo turret;
    boolean levelMode = true;
    boolean clawOpen;

    public enum ClawPos {
        CLOSE(1), OPEN(0.511);
        private final double pos;

        ClawPos(double val) {
            this.pos = val;
        }
        public double getPos() {
            return pos;
        }
    }

    public enum ElbowPos {
        MID(0.45), MAX_DEPOSIT(0.75), MAX_COLLECT(0);
        private final double pos;

        ElbowPos(double val) {
            this.pos = val;
        }
        public double getPos() {
            return pos;
        }
    }
    public enum TurretPos {
        LEFT(0.34), RIGHT(0.75);
        private final double pos;

        TurretPos(double val) {
            this.pos = val;
        }
        public double getPos() {
            return pos;
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(CRServo.class, "claw");
        pivot = hardwareMap.get(CRServo.class, "pivot");
        shoulder = hardwareMap.get(Servo.class, "shoulder"); //312.5
        elbow = hardwareMap.get(Servo.class, "elbow"); // 312.5
        turret = hardwareMap.get(Servo.class, "turret");
        shoulderPos = angleToPos(shoulderMaxPos, shoulderAngle, 90);
        turretPos=1;
        turret.setPosition(1);
//        elbowPos = angleToPos(elbowMaxPos, elbowAngle, 180);
//        pivotPos = angleToPos(pivotMaxPos, pivotAngle, 90);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        ToggleButtonReader bReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );

        waitForStart();
        while (opModeIsActive()) {
            extendMagnitude = Math.pow(gamepad1.right_stick_x, 3);
            turnMagnitude = Math.pow(gamepad1.left_stick_x, 3);
            if (!(shoulderAngle>=90 && extendMagnitude>0 || shoulderAngle<=40 && extendMagnitude<0)) {
                shoulderAngle+=extendMagnitude;
            }

            if (turnMagnitude>0 && turretPos<=1) {
                turretPos += 0.0005;
            }
            else if (turnMagnitude<0 && turretPos>=0) {
                turretPos -= 0.0005;
            }



            if (levelMode) {
                elbowAngle = 180 - (2 * shoulderAngle);
            }

            if (gamepad1.dpad_up) {
                levelMode = false;
                elbowAngle+=0.1;
            }
            else if (gamepad1.dpad_down) {
                levelMode = false;
                elbowAngle-=0.1;
            }

            if (xReader.wasJustReleased()) {
                levelMode = true;
            }


            if (gamepad1.right_bumper) {
               claw.setPower(-0.5);
               pivot.setPower(-0.5);
            }
            else if (gamepad1.left_bumper) {
                claw.setPower(0.5);
                pivot.setPower(0.5);
            }
            else {
                claw.setPower(0);
                pivot.setPower(0);
            }

            if (gamepad1.dpad_left) {
                claw.setPower(0.5);
                pivot.setPower(-0.5);
            }
            else if (gamepad1.dpad_right) {
                claw.setPower(-0.5);
                pivot.setPower(0.5);
            }
            else {
                claw.setPower(0);
                pivot.setPower(0);
            }

            pivotAngle = elbowAngle+shoulderAngle-90;

            shoulderPos = angleToPos(shoulderMaxPos, shoulderAngle, 90);
            elbowPos = angleOfElbowToPos(elbowAngle);
//
            turret.setPosition(turretPos);
            shoulder.setPosition(shoulderPos);
            elbow.setPosition(elbowPos);
//            pivot.setPosition(pivotPos);
//            claw.setPosition(clawPos);
//            shoulder.setPosition(0);
//            elbow.setPosition(1);
//            pivot.setPosition(0.5);
//            claw.setPosition(0.5);
//            turret.setPosition(0);

            xReader.readValue();
            telemetry.addData("Turn Magnitude: ", extendMagnitude);
            telemetry.addData("\nShoulder Angle: ", shoulderAngle);
            telemetry.addData("Shoulder Pos: ", shoulderPos);
            telemetry.addData("\nElbow Angle: ", elbowAngle);
            telemetry.addData("Elbow Pos: ", elbowPos);
            telemetry.addData("\nPivot Angle: ", pivotAngle);
            telemetry.addData("Pivot Pos: ", pivotPos);
            telemetry.addData("\nClaw Pos: ", clawPos);
            telemetry.addData("\nTurret Pos: ", turretPos);
            telemetry.update();
        }

    }


    private double angleToPos(double posAtMax, double angle, double maxAngle) {
        return (angle/maxAngle) * posAtMax;
    }

    private double angleOfElbowToPos(double angle) {
        return 0 + ((angle-100)/-100) * (0.36-0);
    }
}
