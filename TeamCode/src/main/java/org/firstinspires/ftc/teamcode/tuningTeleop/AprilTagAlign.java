package org.firstinspires.ftc.teamcode.tuningTeleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import androidx.core.location.GnssStatusCompat;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.util.TweakedPID;

@TeleOp
@Config
public class AprilTagAlign extends LinearOpMode {
    public static double yP = 0.0128, yI=0.0003, yD=0.001;
    public static double xP = 0.045, xI=0.0003, xD=0.001;
    TweakedPID xControl = new TweakedPID(xP, xI, xD);
    TweakedPID yControl = new TweakedPID(yP, yI, yD);
    TweakedPID headingControl = new TweakedPID(
            MotionPlannerEdit.headingControlEnd.getP(),
            MotionPlannerEdit.headingControlEnd.getI(),
            MotionPlannerEdit.headingControlEnd.getD());


    double kStaticY = 0.22;

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
        double voltage, xError, yError, headingError, currentHeading;
        double xPower, yPower, magnitude, theta, driveTurn;
        double targetHeading = Pika.localizer.getHeading();
        double desiredArea = 0.07;
        waitForStart();
        Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition());
        while (opModeIsActive()) {
            xControl.setPID(xP, xI, xD);
            yControl.setPID(yP, yI, yD);
            Pose3D robotPose = Pika.limelight.detectAprilTag();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            currentHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);
            yError = Pika.limelight.tY;
            xError = -100*(Pika.limelight.targetArea- desiredArea);
            headingError = targetHeading - currentHeading;


            xPower = xControl.calculate(0, xError);
            xPower = (Math.abs(xError)>1) ? xPower + Math.signum(xPower)* MotionPlannerEdit.kStatic_X : 0;

            yPower = yControl.calculate(0, yError);
            yPower = (Math.abs(yError)>1.25) ? Math.signum(yPower)*kStaticY + yPower : 0;

            magnitude = Math.hypot(xPower, yPower);
            driveTurn = headingControl.calculate(0, headingError);
            driveTurn = (Math.abs(headingError)>2) ?
                    Math.signum(driveTurn)*MotionPlannerEdit.kStatic_Turn + driveTurn : 0;

            theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);
            if (Pika.limelight.targetFound)
                Pika.drivetrain.drive(magnitude, theta, driveTurn, Pika.movementPower, voltage);
            else
                Pika.drivetrain.drive(0,0,0,0);
            Pika.localizer.update();
            telemetry.addData("Tx: ", Pika.limelight.tX);
            telemetry.addData("Ty: ", Pika.limelight.tY);
            telemetry.addData("Area: ", Pika.limelight.targetArea);
            telemetry.addData("xError: ", xError);
            telemetry.addData("yError: ", yError);
            telemetry.addData("HeadingError: ", headingError);
            telemetry.addData("xPower: ", xPower);
            telemetry.addData("yPower: ", yPower);
            telemetry.addData("Theta: ", theta);
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.addData("MovementPower: ", Pika.movementPower);
            telemetry.update();
        }
    }
}
