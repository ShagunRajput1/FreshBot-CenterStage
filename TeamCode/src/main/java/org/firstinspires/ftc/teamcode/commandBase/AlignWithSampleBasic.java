package org.firstinspires.ftc.teamcode.commandBase;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

public class AlignWithSampleBasic extends Command {
    double targetHeading, targetX;
    double headingError, targetArea;
    double angle = 180, slideError, tX, tY, currentHeading, driveTurn;
    double targetAreaThreshold = 0.05;
    double angleMeasureTargetAreaThreshold = 0.110;
    // Decreased from 0.125 because did not detect orientation in one run


    boolean isFinished = false;
    PIDController headingControl = new PIDController(0.018, 0.0001, 0);



    @Override
    public void init() {
        isFinished = false;
        headingControl.setIntegrationBounds(-10000000, 10000000);
        this.targetHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);
        Pika.outtakeSlides.pause();
        Pika.outtakeSlides.resetAlignController();
        tX = 0;
        targetArea = 0;
        slideError = 0;
    }
    @Override
    public void update() {
        if (isFinished) {
            return;
        }

        Pika.limelight.getSampleOrientation();
        tX = Pika.limelight.tX;
        tY = Pika.limelight.tY;
        targetArea = Pika.limelight.targetArea;
        currentHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);

        if (targetArea>angleMeasureTargetAreaThreshold) {
            angle = Pika.limelight.angle;
        }

        slideError = tX+9;

        slideError = (Math.abs(slideError)>1.2) ? slideError : 0;
//        xError = targetX- Pika.localizer.getY();
        headingError = targetHeading - currentHeading;



        driveTurn = headingControl.calculate(0, headingError);

        if (targetArea>targetAreaThreshold)
            Pika.outtakeSlides.alignWithSample(slideError);
        else
            Pika.outtakeSlides.extendForSample();

        if (targetArea>targetAreaThreshold && Math.abs(slideError)<1.2) {
            Pika.newClaw.setPivotOrientation(angle);
            Pika.outtakeSlides.freeMove();
            isFinished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void stop() {
        isFinished = true;
    }

    public String getTelemetry() {
        return "Arm: " + Pika.arm.getTelemetry() +
                "\nEndEffectorClaw: " + Pika.newClaw.getTelemetry() +
                "\nAngle: " + angle +
                "\ntX: " + tX +
                "\ntY: " + tY +
                "\nTarget Area: " + targetArea +
                "\nHeadingError: " + headingError +
                "\nDriveTurn: " + driveTurn +
                "\nMovementPower: " + Pika.movementPower +

                "\nSlide Power: " + Pika.outtakeSlides.getTelemetry() +
                "\nSlideError: " + slideError +

                "\nReachedSlides: " + (Math.abs(slideError)<1.2) +
                "\nEnough Area: " + (targetArea>0.115) +
                "\nArm Finished: " + Pika.arm.isFinished();
    }
}
