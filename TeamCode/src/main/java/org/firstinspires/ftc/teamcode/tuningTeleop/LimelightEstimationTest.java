package org.firstinspires.ftc.teamcode.tuningTeleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.AutoGrab;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Camlight;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp
@Config
public class LimelightEstimationTest extends LinearOpMode {
    public static double yP = 0.04, yI = 0.0055, yD=0;
    public static double hP = 0.03, hI = 0.00025, hD=0;

    AutoGrab autoGrab = new AutoGrab();

    PIDController yControl = new PIDController(
            yP,
            yI,
            yD
    );
    PIDController headingControl = new PIDController(
            hP,
            hI,
            hD
    );

    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        yControl.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        Camlight.red = true;
        double targetY = 0;
        boolean oriented = false;
        boolean posMode = false;
        double pivotPos = 180;
        boolean clawOpen = true;
        double pos = FinalClaw.ArmPitch.RETRACT.getPosition();
        double miniPitchPos = FinalClaw.MiniPitch.RETRACT.getPosition();
        double clawPos = FinalClaw.ClawPosition.CLOSE.getPosition();
        int armPos = Arm.ArmPos.INTAKE.getPosition();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        ToggleButtonReader dpad_left_reader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_LEFT
        );
        ToggleButtonReader goToPosition = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );


        waitForStart();

        while (opModeIsActive()) {
            yControl.setPID(yP, yI, yD);
            headingControl.setPID(hP, hI, hD);
            Pika.limelight.getSampleOrientation();

            if (dpad_left_reader.wasJustReleased()) {
                pos = FinalClaw.ArmPitch.NEW_DETECT.getPosition();
                miniPitchPos = FinalClaw.MiniPitch.NEW_DETECT.getPosition();
                Pika.newClaw.setPivotOrientation(pivotPos);
                Pika.newClaw.pitchA.setPosition(pos);
                Pika.newClaw.pitchB.setPosition(1-pos);
                Pika.newClaw.miniPitch.setPosition(miniPitchPos);
                pivotPos = 90;
                Pika.outtakeSlides.resetPID();
                posMode = false;
            }

            if (gamepad1.right_bumper) {
                Pika.outtakeSlides.goUp();
                posMode = false;

            }
            else if (gamepad1.left_bumper) {
                Pika.outtakeSlides.goDown();
                posMode = false;

            }
            else {
                if (!posMode)
                    Pika.outtakeSlides.freeMove();
            }

            if (goToPosition.wasJustReleased()) {
                posMode = true;
//                pivotPos = Pika.limelight.angle;
                Pika.outtakeSlides.setTargetPosition(Pika.limelight.estimatedSlideExtension);
                targetY = Pika.limelight.estimatedStrafe + Pika.localizer.getY();
                yControl.reset();
                oriented = false;
                Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition());
                Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DETECT.getPosition());
            }



            if (xReader.wasJustReleased()) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition());
                }
                else
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
            }


            if (gamepad1.right_trigger>0 && pivotPos<=180) {
                pivotPos+=1;
            }
            else if (gamepad1.left_trigger>0 && pivotPos>0) {
                pivotPos -=1;

            }

            Pika.arm.setTargetPosition(armPos);
            Pika.arm.update();



            double yError = targetY-Pika.localizer.getY();
            double headingError = 0-Pika.localizer.getHeading(Localizer.Angle.DEGREES);
            double driveTurn = headingControl.calculate(0, headingError);
            driveTurn = Math.signum(driveTurn)*MotionPlannerEdit.kStatic_Turn + driveTurn;
            driveTurn = (Math.abs(headingError)>1) ? driveTurn : 0;




            double yPower = yControl.calculate(0, yError);
            yPower = (Math.abs(yError)>0.1) ? Math.signum(yPower)*0.25 + yPower : 0;
            double theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, 0)));
            if (posMode)
                Pika.drivetrain.drive(Math.hypot(yPower, 0), theta, driveTurn, 0.85);
            else
                Pika.drivetrain.drive(0,0,0,0);

            if (posMode && Math.abs(yError)<0.1 && Math.abs(headingError)<1 && Pika.outtakeSlides.isFinished() && !oriented) {
//                pivotPos = Pika.limelight.getSampleOrientation();
                oriented = true;
                yControl.reset();
                headingControl.reset();
                autoGrab.init();
            }
            if (posMode && Math.abs(yError)<0.1) {
                yControl.reset();
                Pika.outtakeSlides.update();
            }
            else {
                if (posMode) {
                    Pika.outtakeSlides.freeMove();
                }
            }

            Pika.localizer.update();

            goToPosition.readValue();
            xReader.readValue();
            dpad_left_reader.readValue();
            telemetry.addData("SlidePos: ", Pika.outtakeSlides.getCurrentPosition());
            telemetry.addData("PitchAPos: ", pos);
            telemetry.addData("PitchBPos: ", (1-pos));
            telemetry.addData("MiniPitchPos: ", miniPitchPos);
            telemetry.addData("claw: ", clawPos);
            telemetry.addData("PivotPos: ", Pika.newClaw.pivotPos);
            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
            telemetry.addData("Localizer: ", Pika.localizer.getTelemetry());
            telemetry.addData("\nLimelight: ", Pika.limelight.getTelemetry());
            telemetry.addData("TargetY: ", targetY);
            telemetry.addData("Oriented: ", oriented);
            telemetry.addLine("\nMovement ");
            telemetry.addData("driveTurn: ", driveTurn);
            telemetry.addData("yPower: ", yPower);
            telemetry.addData("yError: ", yError);
            telemetry.addData("headingError: ", headingError);
            telemetry.addData("Slides: ", Pika.outtakeSlides.getTelemetry());
            autoGrab.update();


            telemetry.update();
        }
    }
}
