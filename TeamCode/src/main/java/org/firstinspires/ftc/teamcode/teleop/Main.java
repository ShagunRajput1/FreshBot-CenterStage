package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.SampleAuto;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.Drop;
import org.firstinspires.ftc.teamcode.commandBase.Grab;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.PrepareOuttake;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

@TeleOp
public class Main extends LinearOpMode {
    boolean pitching = true;
    boolean pivotGrab = true;
    double armTargetPos;
    boolean slidePositionMode = false;
    boolean aligning = false;
    double armPitchPos = Claw.ArmPitch.RETRACT.getPosition();
    int slidePos;

    boolean clawOpen = true;
    boolean autoMovement = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, false);
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
        MotionPlannerEdit mp = new MotionPlannerEdit(Pika.drivetrain, Pika.localizer, hardwareMap);
        PrepareOuttake prepareOuttake = new PrepareOuttake();
        DepositSample highBasket = new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks());
        IntakeSample intakeSample = new IntakeSample();
        RetractAll retract = new RetractAll();
        Grab grab = new Grab();
        Drop drop = new Drop();
        FollowTrajectory goToBucket = new FollowTrajectory(mp, new Bezier(-45, SampleAuto.bucketDeposit));



        while (opModeIsActive()) {
            // Drivetrain stuff
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 1);
            driveTurn = (Math.abs(driveTurn) > 0) ?
                    (driveTurn + Math.signum(driveTurn)* MotionPlannerEdit.kStatic_Turn) : 0;
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
                Pika.drivetrain.drive(magnitude, theta, driveTurn, Pika.movementPower);

            if (alignReader.wasJustReleased()) {
                goToBucket.init();
                autoMovement = true;
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
                    prepareOuttake.init();
                }
                else {
                    if (Pika.arm.isFinished()) {

                        Pika.outtakeSlides.setPower(0.75);
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
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition()) {
                    slidePositionMode = true;
                    Pika.outtakeSlides.holdSlides();
                    Pika.outtakeSlides.resetPID();
                }
            }



            if (gamepad1.dpad_left) {
                if (!readyToIntake()) {
                    Pika.movementPower = 0.3;
                    slidePositionMode = true;
                    intakeSample.init();
                    retract.stop();
                    prepareOuttake.stop();
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

            if (aligning) {
                Pika.newClaw.setPivotOrientation(Pika.limelight.getSampleOrientation());
            }

            if (yReader.wasJustReleased()) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    if (Pika.newClaw.clawPos == FinalClaw.ClawPosition.CLOSE.getPosition()) {
                        grab.stop();
                        Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition());
                    }
                    else {
                        grab.init();
                    }
                }
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() &&
                        Pika.arm.isFinished()) {
                    drop.init();
                }

            }


            if (aReader.wasJustReleased()) {
                intakeSample.stop();
                prepareOuttake.stop();
                grab.stop();
                retract.init();
                slidePositionMode = true;
                Pika.movementPower = 0.8;
            }


            if (gamepad1.dpad_up) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.newClaw.pitchPos += 0.005;
                }
                else {
                    retract.stop();
                    intakeSample.stop();
                    prepareOuttake.stop();
                    slidePositionMode = true;
                    Pika.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.HANG.getTicks());
                    Pika.arm.setTargetPosition(Arm.ArmPos.HANG.getPosition());
                }

            }
            else if (gamepad1.dpad_down) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.newClaw.pitchPos-= 0.005;
                }
                else {
                    retract.stop();
                    intakeSample.stop();
                    prepareOuttake.stop();
                    slidePositionMode = true;
                    Pika.outtakeSlides.setTargetPosition(OuttakeSlides.TurnValue.HANG_RETRACT.getTicks());
                    Pika.arm.setTargetPosition(Arm.ArmPos.HANG.getPosition());
                }
            }
//            Pika.newClaw.setArmPitch(Pika.claw.pitchPos);
//            Pika.claw.armPitchSupplement.setPosition(1-armPitchPos);

            if (gamepad1.right_trigger>0 && Pika.newClaw.orientation<=180) {
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation+10);
            }
            else if (gamepad1.left_trigger>0 && Pika.newClaw.orientation>=0) {
                Pika.newClaw.setPivotOrientation(Pika.newClaw.orientation-10);
            }

            if (!Pika.arm.isFinished()) {
                Pika.outtakeSlides.freeMove();
            }
            else {
                if (slidePositionMode)
                    Pika.outtakeSlides.update();
            }

            drop.update();
            grab.update();
            intakeSample.update();
            prepareOuttake.update();
            retract.update();
            Pika.arm.update();
            Pika.localizer.update();
            alignReader.readValue();
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
            telemetry.update();
        }
        // 14480
        // 9587

    }


    private boolean readyToIntake() {
        return Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()
                && Pika.arm.isFinished() && Pika.newClaw.pitchPos == FinalClaw.ArmPitch.BEFORE_GRAB.getPosition();
    }

}
