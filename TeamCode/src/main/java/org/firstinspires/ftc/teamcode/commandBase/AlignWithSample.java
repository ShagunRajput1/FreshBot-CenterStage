package org.firstinspires.ftc.teamcode.commandBase;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

public class AlignWithSample extends Command {
    double targetHeading, targetX;
    double xError, yError, headingError, targetArea;
    double kStaticY = 0.2;
    PIDController yControl = new PIDController(0.0145, 0.0002, 0.0001);
    PIDController xControl = new PIDController(0.02, 0.0015, 0.01);
    PIDController headingControl = new PIDController(0.01, 0.0001, 0);
    double error;
    @Override
    public void init() {
        yControl.setIntegrationBounds(-10000000, 10000000);
        xControl.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        this.targetX = Pika.localizer.getX();
        this.targetHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);
    }
    @Override
    public void update() {
        double angle, slideError, tX, currentHeading, xPower, yPower, driveTurn, theta;

        angle = Pika.limelight.getSampleOrientation();
        Pika.newClaw.setPivotOrientation(angle);
        slideError = Pika.limelight.tY;
        slideError = (Math.abs(slideError)>0.5) ? slideError : 0;

        tX = Pika.limelight.tX;
        targetArea = Pika.limelight.targetArea;
        yError = (targetArea>0.15) ? -tX : 0;


        xError = targetX- Pika.localizer.getX();
        currentHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);
        headingError = targetHeading - currentHeading;

        xPower = xControl.calculate(0, xError);
        xPower = (Math.abs(xError)>1) ? xPower + Math.signum(xPower)* MotionPlannerEdit.kStatic_X : 0;

        driveTurn = headingControl.calculate(0, headingError);


        yPower = yControl.calculate(0, yError);
        yPower = (Math.abs(yError)>0.7) ? Math.signum(yPower)*kStaticY + yPower : 0;

        theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);

        if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()
                && targetArea>0.15 && Math.abs(slideError)<0.5 && Math.abs(yError)<0.7) {
            Pika.outtakeSlides.retractToIntake();
        }

        Pika.drivetrain.drive(Math.hypot(yPower, xPower), theta, driveTurn, Pika.movementPower);

    }

    @Override
    public boolean isFinished() {
        return (xError<=1 && yError<=0.7 && targetArea>0.15);
    }
}
