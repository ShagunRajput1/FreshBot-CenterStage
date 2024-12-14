package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.SampleAuto;
import org.firstinspires.ftc.teamcode.commandBase.Drop;
import org.firstinspires.ftc.teamcode.commandBase.GrabSpec;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleSpec;
import org.firstinspires.ftc.teamcode.commandBase.PrepareForDeposit;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeBasket;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeSpec;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
import org.firstinspires.ftc.teamcode.commandBase.TeleGrab;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.pathing.Point;
import org.firstinspires.ftc.teamcode.util.TweakedPID;

@TeleOp
public class Main extends LinearOpMode {
    boolean slidePositionMode = false;
    boolean aligning = false;
    boolean clawOpen = true;
    boolean autoMovement = false;
    double targetHeading, headingError;
    boolean holdingHeading = false;
    boolean bucketMode = true;
    ElapsedTime headingTimer = new ElapsedTime();
    Point bucketDeposit = new Point(9, 13);
    public static TweakedPID headingControl = new TweakedPID(
            MotionPlannerEdit.headingControlEnd.getP(),
            MotionPlannerEdit.headingControlEnd.getI(),
            MotionPlannerEdit.headingControlEnd.getD()
    );


    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, true);
        Pika.localizer.setX(SampleAuto.bucketDeposit.getX());
        Pika.localizer.setY(SampleAuto.bucketDeposit.getY());
        waitForStart();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx driverOp2 = new GamepadEx(gamepad2);
        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );
        ToggleButtonReader yReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
        ToggleButtonReader bReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );
        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        ToggleButtonReader alignReader = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );
        ToggleButtonReader modeReader = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.X
        );
        MotionPlannerEdit mp = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);
        PrepareOuttakeBasket prepareOuttake = new PrepareOuttakeBasket();
        PrepareOuttakeSpec prepareOuttakeSpec = new PrepareOuttakeSpec();
        PrepareForDeposit highBasket = new PrepareForDeposit();
        IntakeSample intakeSample = new IntakeSample();
        IntakeSampleSpec intakeSampleSpec = new IntakeSampleSpec();
        RetractAll retract = new RetractAll();
        TeleGrab grab = new TeleGrab();
        GrabSpec grabSpec = new GrabSpec();
        Drop drop = new Drop();
        FollowTrajectory goToBucket = new FollowTrajectory(mp, new Bezier(-45, bucketDeposit));
        headingControl.setIntegrationBounds(-10000000, 10000000);



        while (opModeIsActive()) {
            // Drivetrain stuff
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 1);
            driveTurn = (Math.abs(driveTurn) > 0) ?
                    (driveTurn + Math.signum(driveTurn)* MotionPlannerEdit.kStatic_Turn) : 0;
            if (driveTurn == 0) {
                if (!holdingHeading) {
                    headingTimer.reset();
                    headingControl.reset();
                    holdingHeading = true;
                }
                if (headingTimer.seconds()>1.5) {
                    headingError = targetHeading - Pika.localizer.getHeading(Localizer.Angle.DEGREES);
                    driveTurn = (Math.abs(headingError) > 1) ? headingControl.calculate(0, headingError) : 0;
                    driveTurn = driveTurn + Math.signum(driveTurn) * MotionPlannerEdit.kStatic_Turn;
                }
                else {
                    targetHeading = Pika.localizer.getHeading(Localizer.Angle.DEGREES);
                }
            }
            else {
                holdingHeading = false;
            }
            double driveY = Math.pow(-gamepad2.left_stick_x, 1);
            driveY = (Math.abs(driveY) > 0) ?
                    driveY + Math.signum(driveY)*MotionPlannerEdit.kStatic_Y : 0;
            double driveX = Math.pow(-gamepad2.left_stick_y, 1);
            driveX = (Math.abs(driveX) > 0) ?
                    driveX + Math.signum(driveX)*MotionPlannerEdit.kStatic_X : 0;

            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            double movementPower = Pika.movementPower;
            if (!autoMovement)
                Pika.drivetrain.drive(magnitude, theta, driveTurn, Pika.movementPower, hardwareMap.voltageSensor.iterator().next().getVoltage());

            if (alignReader.wasJustReleased()) {
                goToBucket.init();
                autoMovement = true;
            }
            if (modeReader.wasJustReleased()) {
                bucketMode = !bucketMode;
            }

            if (autoMovement){
                mp.update();
                if (mp.isFinished()) {
                    autoMovement = false;
                }
            }

            if (gamepad1.right_bumper) {
                Pika.movementPower = 0.3;
                if (Pika.arm.getTargetPosition() != Arm.ArmPos.OUTTAKE.getPosition()) {
                    slidePositionMode = true;
                    if (bucketMode)
                        prepareOuttake.init();
                    else
                        prepareOuttakeSpec.init();
                }
                else {
                    if (Pika.arm.isFinished()) {

                        Pika.outtakeSlides.setPower(0.7);
                        slidePositionMode = false;
                        prepareOuttake.stop();
                        intakeSample.stop();
                        highBasket.stop();
                        retract.stop();
                        Pika.outtakeSlides.goUp();
                    }
                }
            }
            else if (gamepad1.left_bumper &&
                    (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition())) {
                slidePositionMode = false;
                retract.stop();
                prepareOuttake.stop();
                intakeSample.stop();
                highBasket.stop();
                Pika.outtakeSlides.goDown();
            }
            else {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() && !slidePositionMode) {
                    slidePositionMode = true;
                    Pika.outtakeSlides.holdSlides();
                }
            }



            if (gamepad1.dpad_left) {
                if (!readyToIntake()) {
                    Pika.movementPower = 0.3;
                    slidePositionMode = true;
                    if (bucketMode)
                        intakeSample.init();
                    else
                        intakeSampleSpec.init();
                    retract.stop();
                    prepareOuttake.stop();
                    highBasket.stop();
                }
                else {
                    if (Pika.arm.isFinished()) {
                        Pika.outtakeSlides.setPower(0.4);
                        slidePositionMode = false;
                        Pika.outtakeSlides.goUp();
                    }
                }
            }
            else if (gamepad1.dpad_right) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.outtakeSlides.goDown();
                    slidePositionMode = false;
                }
            }
            else {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() && !slidePositionMode)
                    Pika.outtakeSlides.stopSlides();
            }



            if (xReader.wasJustReleased()) {
                slidePositionMode = true;
                Pika.outtakeSlides.retractToIntake();
            }

//            if (yReader.wasJustReleased()) {
//                double[] positions;
//                pivotGrab = !pivotGrab;
//                if (pivotGrab) {
//                    positions = Claw.PitchPosition.PLACE.getPosition();
//                    Pika.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks());
//
//                }
//                else {
//                    positions = Claw.PitchPosition.GRAB.getPosition();
//                }
//                Pika.claw.setPitch(positions[0], positions[1]);
//            }
            if (bReader.wasJustReleased()) {
                aligning = !aligning;
            }

            if (aligning && readyToIntake()) {
                Pika.newClaw.setPivotOrientation(Pika.limelight.getSampleOrientation());
            }

            if (yReader.wasJustReleased()) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.CLOSE.getPosition()) {
                        grab.stop();
                        Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                    }
                    else {
                        aligning = false;
                        if (bucketMode)
                            grab.init();
                        else
                            grabSpec.init();
                    }
                }
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() &&
                        Pika.arm.isFinished()) {
                    Pika.localizer.setX(SampleAuto.bucketDeposit.getX());
                    Pika.localizer.setY(SampleAuto.bucketDeposit.getY());
                    if (bucketMode)
                        drop.init();
                    else {
                        if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.CLOSE.getPosition()) {
                            grab.stop();
                            Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                        }
                    }
                }

            }


            if (aReader.wasJustReleased()) {
                intakeSample.stop();
                prepareOuttake.stop();
                grab.stop();
                retract.init();
                highBasket.stop();
                slidePositionMode = true;
                Pika.movementPower = 0.8;
            }


            if (gamepad1.dpad_up) {
                Pika.movementPower = 0.3;
                slidePositionMode = true;
                highBasket.init();
                intakeSample.stop();
                prepareOuttake.stop();
                grab.stop();
                retract.stop();
            }


            if (gamepad1.right_trigger>0 && Pika.newClaw.orientation<=180) {
                aligning = false;
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation+(15*gamepad1.right_trigger));
            }
            else if (gamepad1.left_trigger>0 && Pika.newClaw.orientation>=0) {
                aligning = false;
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation-(15*gamepad1.left_trigger));
            }

            if (!Pika.arm.isFinished()) {
                Pika.outtakeSlides.freeMove();
            }
            else {
                if (slidePositionMode)
                    Pika.outtakeSlides.update();
            }

            drop.update();
            highBasket.update();
            prepareOuttakeSpec.update();
            grab.update();
            grabSpec.update();
            intakeSample.update();
            prepareOuttake.update();
            intakeSampleSpec.update();
            retract.update();
            Pika.arm.update();
            Pika.localizer.update();
            alignReader.readValue();
            modeReader.readValue();
            xReader.readValue();
            yReader.readValue();
            bReader.readValue();
            aReader.readValue();
            telemetry.addData("DriveTrain current: ", Pika.drivetrain.totalCurrent());
            telemetry.addData("SlideCurrent: ", Pika.outtakeSlides.totalCurrent());
            telemetry.addData("ArmCurrent: ", Pika.arm.totalCurrent());
            telemetry.addData("SlidePositionMode: ", slidePositionMode);
            telemetry.addData("\nClawOpen: ", clawOpen);
            telemetry.addData("\nX: ", Pika.localizer.getX());
            telemetry.addData("\nY: ", Pika.localizer.getY());
            telemetry.addData("Tx: ", Pika.limelight.tX);
            telemetry.addData("Ty: ", Pika.limelight.tY);
            telemetry.addData("Tx: ", Pika.limelight.tX);
            telemetry.addData("Ty: ", Pika.limelight.tY);
            telemetry.addData("\nArm: \n", Pika.arm.getTelemetry());
            telemetry.addData("\nClaw: \n", Pika.newClaw.getTelemetry());
            telemetry.addData("\nSlides: \n", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("", Pika.localizer.getTelemetry());
            telemetry.addData("Bruh: ", Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
                    Pika.outtakeSlides.getCurrentPosition() < OuttakeSlides.TurnValue.MAX_EXTENSION_DOWN.getTicks());
            telemetry.addData("BucketMode: ", bucketMode);
            telemetry.update();
        }
        // 14480
        // 9587

    }


    private boolean readyToIntake() {
        return Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()
                && Pika.arm.isFinished() &&
                (Pika.newClaw.pitchPos == FinalClaw.ArmPitch.BEFORE_GRAB.getPosition() ||
                        Pika.newClaw.pitchPos == FinalClaw.ArmPitch.SPEC_GRAB.getPosition());
    }

}
