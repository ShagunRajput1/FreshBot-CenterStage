package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.ActualTeleOpHang;
import org.firstinspires.ftc.teamcode.commandBase.AutoIntakeFromSubBasic;
import org.firstinspires.ftc.teamcode.commandBase.Drop;
import org.firstinspires.ftc.teamcode.commandBase.GrabSpec;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleSpec;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSampleTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.PrepareForDepositTele;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeBasket;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttakeSpec;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
import org.firstinspires.ftc.teamcode.commandBase.SpecDepositTeleOp;
import org.firstinspires.ftc.teamcode.commandBase.TeleGrab;
import org.firstinspires.ftc.teamcode.commandBase.Unhang;
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

@Config
@TeleOp
public class Main extends LinearOpMode {

    public static double movementPower = 0.7;
    public static double driveTurnMultiplier = 0.75;
    boolean slidePositionMode = false;
    boolean autoOrient = false;
    double targetHeading, headingError;
    boolean holdingHeading = false;
    boolean bucketMode = true;
    ElapsedTime headingTimer = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    Point bucketDeposit = new Point(9, 13);
    public static TweakedPID headingControl = new TweakedPID(
            MotionPlannerEdit.headingControlEnd.getP(),
            MotionPlannerEdit.headingControlEnd.getI(),
            MotionPlannerEdit.headingControlEnd.getD()
    );
    MotionPlannerEdit mp = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);
    PrepareOuttakeBasket prepareOuttake = new PrepareOuttakeBasket();
    PrepareOuttakeSpec prepareOuttakeSpec = new PrepareOuttakeSpec();
    PrepareForDepositTele highBasket = new PrepareForDepositTele();
    IntakeSampleTeleOp intakeSample = new IntakeSampleTeleOp();
    IntakeSampleSpec intakeSampleSpec = new IntakeSampleSpec();
    RetractAll retract = new RetractAll();
    AutoIntakeFromSubBasic autoGrab = new AutoIntakeFromSubBasic();
    TeleGrab grab = new TeleGrab();
    GrabSpec grabSpec = new GrabSpec();
    Drop drop = new Drop();
    ActualTeleOpHang hang = new ActualTeleOpHang();
    Unhang unhang = new Unhang();
    SpecDepositTeleOp depositSpec = new SpecDepositTeleOp();


    @Override
    public void runOpMode() throws InterruptedException {
//        Pika.localizer.setX(SampleAuto.bucketDeposit.getX());
//        Pika.localizer.setY(SampleAuto.bucketDeposit.getY());
        waitForStart();
        Pika.init(hardwareMap, this, true);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx driverOp2 = new GamepadEx(gamepad2);
        
        ToggleButtonReader hangButton = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.X
        );

        ToggleButtonReader autoOrientButton = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );

        ToggleButtonReader clawButton = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.Y
        );
       
        ToggleButtonReader retractButton = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );
        ToggleButtonReader alignReader = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );
        ToggleButtonReader modeButton = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.X
        );

        ToggleButtonReader dpadUp = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.DPAD_UP
        );
        
        FollowTrajectory goToBucket = new FollowTrajectory(mp, new Bezier(-45, bucketDeposit));
        headingControl.setIntegrationBounds(-10000000, 10000000);
        hangTimer.reset();
        slidePositionMode = true;

        
        while (opModeIsActive()) {
            // Drivetrain stuff
            double driveTurn = driveTurnMultiplier* Math.pow(-gamepad2.right_stick_x, 3);
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
            Pika.movementPower = movementPower;
            Pika.drivetrain.drive(magnitude, theta, driveTurn, Pika.movementPower, Pika.getVoltage());

            if (gamepad2.dpad_left) {
                slidePositionMode = false;
//                Pika.outtakeSlides.forceRetract();
            }


            if (modeButton.wasJustReleased()) {
                bucketMode = !bucketMode;
            }


            if (gamepad1.right_bumper) {
                prepareOuttakeAndExtend();
            }
            else if (gamepad1.left_bumper &&
                    (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition())) {
                outtakeRetract();
            }
            else {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() && !slidePositionMode) {
                    slidePositionMode = true;
                    Pika.outtakeSlides.holdSlides();
                }
            }



            if (gamepad1.dpad_left) {
                Pika.movementPower = 0.5;
                if (!readyToIntake()) {
                    slidePositionMode = true;
                    if (bucketMode) {
                        intakeSample.init();
                    }
                    else
                        intakeSampleSpec.init();
                    retract.stop();
                    prepareOuttake.stop();
                    highBasket.stop();
                }
                else {
                    if (Pika.arm.isFinished()) {
                        Pika.outtakeSlides.setPower(0.7);
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

            if (hangButton.wasJustReleased()) {
                if (hangTimer.seconds() > 110) {
                    if (Pika.arm.getTargetPosition() != Arm.ArmPos.PREP_HANG.getPosition()) {
                        slidePositionMode = true;
                        hang.init();
                    }
                    if (Pika.arm.getTargetPosition() == Arm.ArmPos.HANG.getPosition()) {
                        unhang.init();
                        hang.stop();
                        slidePositionMode = true;
                    }
                }
            }

//            if (limelightAlignmentButton.wasJustReleased()) {
//                if (aligning)
//                    alignWithSample.stop();
//                else
//                    alignWithSample.init();
//                aligning = !aligning;
//            }


            if (clawButton.wasJustReleased()) {
                grabAndDrop();
            }


            if (retractButton.wasJustReleased()) {
                intakeSample.stop();
                prepareOuttake.stop();
                autoGrab.stop();
                if (!bucketMode) {
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                }
                retract.init();
                highBasket.stop();
                slidePositionMode = true;
                Pika.movementPower = 0.8;
            }


            if (dpadUp.wasJustReleased()) {
                Pika.movementPower = 0.5;
                slidePositionMode = true;
                if (bucketMode)
                    highBasket.init();
                else {
                    if (Pika.outtakeSlides.getTargetPosition() != OuttakeSlides.TurnValue.SPEC_PREP.getTicks()) {
                        prepareOuttakeSpec.init();
                    }
                    else {
                        depositSpec.init();
                    }
                }
                intakeSample.stop();
                prepareOuttake.stop();
                autoGrab.stop();
                retract.stop();
            }

            if (autoOrientButton.wasJustReleased()) {
                autoOrient = !autoOrient;
            }


            if (gamepad1.right_trigger>0 && Pika.newClaw.orientation<=180) {
                autoOrient = false;
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation+(10*gamepad1.right_trigger));
            }
            else if (gamepad1.left_trigger>0 && Pika.newClaw.orientation>=0) {
                autoOrient = false;
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation-(10*gamepad1.left_trigger));
            }

            if (!Pika.arm.isFinished() && Pika.arm.getTargetPosition() != Arm.ArmPos.HANG.getPosition()) {
                Pika.outtakeSlides.freeMove();
            }
            else {
                if (slidePositionMode)
                    Pika.outtakeSlides.update();
            }

//            Fix this failsafe
//            if (!retract.isFinished() && Pika.outtakeSlides.isStuck()) {
//                retract.stop();
//            }


            updateAllCommands();
            Pika.arm.update();
            Pika.localizer.update();
            alignReader.readValue();
            modeButton.readValue();
            hangButton.readValue();
            clawButton.readValue();
            autoOrientButton.readValue();
            dpadUp.readValue();
            retractButton.readValue();
//            telemetry.addData("DriveTrain current: ", Pika.drivetrain.totalCurrent());
            telemetry.addData("SlideCurrent: ", Pika.outtakeSlides.totalCurrent());
//            telemetry.addData("ArmCurrent: ", Pika.arm.totalCurrent());
            telemetry.addData("SlidePositionMode: ", slidePositionMode);
//            telemetry.addData("\nClawOpen: ", clawOpen);
            telemetry.addData("\nX: ", Pika.localizer.getX());
            telemetry.addData("\nY: ", Pika.localizer.getY());
//            telemetry.addData("Tx: ", Pika.limelight.tX);
//            telemetry.addData("Ty: ", Pika.limelight.tY);
//            telemetry.addData("Tx: ", Pika.limelight.tX);
//            telemetry.addData("Ty: ", Pika.limelight.tY);
//            telemetry.addData("\nArm: \n", Pika.arm.getTelemetry());
//            telemetry.addData("\nClaw: \n", Pika.newClaw.getTelemetry());
            telemetry.addData("\nSlides: \n", Pika.outtakeSlides.getTelemetry());
//            telemetry.addData("", Pika.localizer.getTelemetry());
//            telemetry.addData("Bruh: ", Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
//                    Pika.outtakeSlides.getCurrentPosition() < OuttakeSlides.TurnValue.MAX_EXTENSION_DOWN.getTicks());
            telemetry.addData("BucketMode: ", bucketMode);
            telemetry.addData("auto orient: ", autoOrient);

            telemetry.update();



        }
        // 14480
        // 9587

    }
    

    private boolean readyToIntake() {
        return Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()
                && Pika.arm.isFinished() &&
                ((bucketMode && (Pika.newClaw.pitchPos == FinalClaw.ArmPitch.BEFORE_GRAB.getPosition() ||
                        Pika.newClaw.pitchPos == FinalClaw.ArmPitch.GRAB.getPosition()) &&
                        (Pika.newClaw.orientation == 90 || !autoOrient)) ||
                (!bucketMode && (Pika.newClaw.pitchPos ==  FinalClaw.ArmPitch.SPEC_GRAB.getPosition())));
    }
    
    private void prepareOuttakeAndExtend() {
        Pika.movementPower = 0.575;
        if (Pika.arm.getTargetPosition() != Arm.ArmPos.OUTTAKE.getPosition()) {
            slidePositionMode = true;
            if (bucketMode)
                prepareOuttake.init();
            else
                prepareOuttakeSpec.init();
        }
        else {
            if (Pika.arm.isFinished()) {
                Pika.outtakeSlides.setPower(1);
                slidePositionMode = false;
                prepareOuttake.stop();
                intakeSample.stop();
                highBasket.stop();
                retract.stop();
                Pika.outtakeSlides.goUp();
            }
        }
    }
    
    private void outtakeRetract() {
        slidePositionMode = false;
        retract.stop();
        prepareOuttake.stop();
        intakeSample.stop();
        highBasket.stop();
        Pika.outtakeSlides.goDown();
    }

    private void grabAndDrop() {
        if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
            if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.CLOSE.getPosition()) {
                autoGrab.stop();
                Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
            }
            else {
                if (bucketMode) {
                    if (autoOrient)
                        autoGrab.init();
                    else
                        grab.init();
                }
                else
                    grabSpec.init();
            }
        }
        if (Pika.arm.getTargetPosition() == Arm.ArmPos.TELEOP_DEPOSIT.getPosition() &&
                Pika.arm.isFinished()) {
            if (bucketMode)
                drop.init();
            else {
                if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.CLOSE.getPosition()) {
                    autoGrab.stop();
                    Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                }
            }
        }

    }

    private void updateAllCommands() {
        drop.update();
        highBasket.update();
        prepareOuttakeSpec.update();
        autoGrab.update();
        grab.update();
        grabSpec.update();
        intakeSample.update();
        prepareOuttake.update();
        intakeSampleSpec.update();
        hang.update();
        unhang.update();
//        alignWithSample.update();
        retract.update();
        depositSpec.update();
    }

}
