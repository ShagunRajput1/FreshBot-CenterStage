package org.firstinspires.ftc.teamcode.tuningTeleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import androidx.core.util.SparseIntArrayKt;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.util.TweakedPID;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp
@Config
public class AlignWithSample extends LinearOpMode {
    public static double P = 0.0128, I=0.0003, D=0.001;
    public static double slideP = 0.034, slideI=0.005, slideD;
    public static double kStaticY = 0.22;

    TweakedPID alignControl = new TweakedPID(P, I, D);
    PIDController xControl = new PIDController(
            MotionPlannerEdit.translationalControlEndX.getP(),
            MotionPlannerEdit.translationalControlEndX.getI(),
            MotionPlannerEdit.translationalControlEndX.getD());
    PIDController headingControl = new PIDController(
            MotionPlannerEdit.headingControlEnd.getP(),
            MotionPlannerEdit.headingControlEnd.getI(),
            MotionPlannerEdit.headingControlEnd.getD());

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        IntakeSample intakeSample = new IntakeSample();
        Grab grab = new Grab();
        double voltage = 0;
        double pivotAngle = 90;
        final double targetAreaThreshold = 0.1;
        double slideError;
        boolean isAligned = false;
        boolean startAligning = false;
        boolean slidePositionMode = false;
        boolean autoSlide = false;
        double yError, angle = 0, tX, tY, targetArea;
        double xError, headingError;
        double targetX = Pika.localizer.getX();
        double currentHeading;
        double targetHeading = Pika.localizer.getHeading();
        double yPower, xPower, driveTurn, theta;
        Pika.movementPower = 0.8;
        alignControl.setIntegrationBounds(-10000000, 10000000);
        waitForStart();

        while (opModeIsActive()) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            alignControl.setPID(P, I, D);

            if (gamepad1.dpad_left) {
                startAligning = true;
                if (Pika.arm.getTargetPosition() != Arm.ArmPos.INTAKE.getPosition()) {
                    intakeSample.init();
                    slidePositionMode = true;
                }
                else {
                    if (intakeSample.isFinished()) {            // Changed but not tested
                        Pika.outtakeSlides.setPower(0.4);
                        Pika.outtakeSlides.goUp();
                    }
                    slidePositionMode = false;
                }
            }
            else if (gamepad1.dpad_right) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.outtakeSlides.goDown();
                }
                slidePositionMode = false;
            }
            else {
                slidePositionMode = true;
                Pika.outtakeSlides.holdSlides();
            }
            Pika.outtakeSlides.sampleSetPID(slideP, slideI, slideD);

            if (startAligning && intakeSample.isFinished() &&
                    Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition())                  // Changed but not tested
                angle = Pika.limelight.getSampleOrientation();


            tX = Pika.limelight.tX;
            tY = Pika.limelight.tY;
            targetArea = Pika.limelight.targetArea;
            currentHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);


            slideError = tX;

//            if (targetArea>0.115 && startAligning && Pika.arm.isFinished()) {
//                pivotAngle = angle;
//            }
//            else {
//                if (startAligning) {
//                    slideError = 6;
//                }
//                else {
//                    slideError = 0;
//                }
//                angle = 90;
//            }


            slideError = (Math.abs(slideError)>1.2) ? slideError : 0;
            xError = targetX- Pika.localizer.getX();
            yError = tY;
            headingError = targetHeading - currentHeading;


            xPower = xControl.calculate(0, xError);
            xPower = (Math.abs(xError)>1) ? xPower + Math.signum(xPower)* MotionPlannerEdit.kStatic_X : 0;

            yPower = alignControl.calculate(0, yError);
            yPower = (Math.abs(yError)>1.25) ? Math.signum(yPower)*kStaticY + yPower : 0;

            driveTurn = headingControl.calculate(0, headingError);

            theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);

            Pika.newClaw.setPivotOrientation(pivotAngle);


            if (slidePositionMode) {            // If not using manual control?
                if (startAligning) {             // If using the auto align method
                    if (targetArea>targetAreaThreshold)
                        Pika.outtakeSlides.alignWithSample(slideError);
                    else
                        Pika.outtakeSlides.extendForSample();
                }
                else
                    Pika.outtakeSlides.update();
            }

            if (intakeSample.isFinished() && targetArea>0.115 &&
                Math.abs(slideError)<1.2 && Math.abs(yError)<1.25 && startAligning) {
                // Condition modified but not tested
                Pika.outtakeSlides.setTargetPosition(Math.max(Pika.outtakeSlides.getCurrentPosition()- OuttakeSlides.retractAmount,
                        OuttakeSlides.TurnValue.RETRACTED.getTicks()));
                grab.init();
                startAligning = false;
            }
            if (intakeSample.isFinished() && startAligning)
                Pika.drivetrain.drive(Math.hypot(yPower, xPower), theta, driveTurn, Pika.movementPower, voltage);
            else
                Pika.drivetrain.drive(0,0,0,0);

            grab.update();
            Pika.arm.update();
            intakeSample.update();
            Pika.localizer.update();
            telemetry.addData("Arm: ", Pika.arm.getTelemetry());
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Error: ", yError);
            telemetry.addData("tX: ", tX);
            telemetry.addData("tY: ", tY);
            telemetry.addData("Target Area: ", targetArea);
            telemetry.addData("xError: ", xError);
            telemetry.addData("HeadingError: ", headingError);
            telemetry.addData("xPower: ", xPower);
            telemetry.addData("yPower: ", yPower);
            telemetry.addData("Theta: ", theta);
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.addData("MovementPower: ", Pika.movementPower);
            telemetry.addData("IntakeDone: ", intakeSample.isFinished());

            telemetry.addData("\nSlide Power: ", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("SlideError: ", slideError);

            telemetry.addData("\nStartAligning: ", startAligning);
            telemetry.addData("ReachedSlides: ", Math.abs(slideError)<1.2);
            telemetry.addData("Reached Y: ", Math.abs(yError)<1.25);
            telemetry.addData("Enough Area: ", targetArea>0.115);
            telemetry.addData("Arm Finished: ", Pika.arm.isFinished());
            telemetry.addData("SlidesFinished: ", Pika.outtakeSlides.isFinished());
            telemetry.addData("Permissible SlideError: ", Pika.outtakeSlides.ERROR);

            telemetry.update();

        }
    }
}
