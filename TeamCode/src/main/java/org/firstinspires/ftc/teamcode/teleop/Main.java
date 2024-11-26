package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.PathFollowerTest;
import org.firstinspires.ftc.teamcode.commandBase.DepositSample;
import org.firstinspires.ftc.teamcode.commandBase.IntakeSample;
import org.firstinspires.ftc.teamcode.commandBase.RetractAll;
import org.firstinspires.ftc.teamcode.commandSystem.FollowTrajectory;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
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
    double armPitchPosS = (1-Claw.ArmPitch.RETRACT.getPosition());
    int slidePos;

    boolean clawOpen = true;
    boolean autoMovement = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pika.init(hardwareMap, this, true);
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
        DepositSample prepareOuttake = new DepositSample(OuttakeSlides.TurnValue.TEST.getTicks());
        DepositSample highBasket = new DepositSample(OuttakeSlides.TurnValue.BUCKET2.getTicks());
        IntakeSample intakeSample = new IntakeSample();
        RetractAll retract = new RetractAll();
        FollowTrajectory goToBucket = new FollowTrajectory(mp, new Bezier(-45, PathFollowerTest.bucketDeposit));



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
                if (!slidePositionMode) {
                    slidePositionMode = true;
                    Pika.outtakeSlides.holdSlides();
                }
            }



            if (gamepad1.dpad_left) {
                if (Pika.arm.getTargetPosition() != Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.movementPower = 0.5;
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
                Pika.claw.setPivotOrientation(Pika.limelight.getSampleOrientation());
            }

//            if (yReader.wasJustReleased()) {
//                intakeSample.stop();
//                prepareOuttake.stop();
//                slidePositionMode = true;
//                highBasket.init();
//            }
            if (yReader.wasJustReleased()) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition()) {
//                    Pika.claw.setArmPitch(Claw.ArmPitch.DEPOSIT.getPosition());
                }
                if (clawOpen)
                    Pika.claw.setClaw(Claw.ClawPosition.OPEN.getPosition());
                else
                    Pika.claw.setClaw(Claw.ClawPosition.CLOSE.getPosition());
                clawOpen = !clawOpen;
            }

            if (aReader.wasJustReleased()) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
                    Pika.claw.clawPos == Claw.ClawPosition.OPEN.getPosition()) {
                    aligning = false;
                    Pika.outtakeSlides.retractToIntake();
                }
                else if ((Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
                        Pika.claw.clawPos == Claw.ClawPosition.CLOSE.getPosition()) ||
                        (Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition())) {
                    slidePositionMode = true;
                    retract.init();
                    prepareOuttake.stop();
                    intakeSample.stop();
                }
            }


            if (gamepad1.dpad_up) {
                if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                    Pika.claw.pitchPos += 0.005;
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
                    Pika.claw.pitchPos-= 0.005;
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
            Pika.claw.armPitch.setPosition(Pika.claw.pitchPos);
//            Pika.claw.armPitchSupplement.setPosition(1-armPitchPos);

            if (gamepad1.right_trigger>0 && Pika.claw.orientation<=180) {
                Pika.claw.setPivotOrientation(Pika.claw.orientation+0.35);
            }
            else if (gamepad1.left_trigger>0 && Pika.claw.orientation>=0) {
                Pika.claw.setPivotOrientation(Pika.claw.orientation-0.35);
            }

            if (!Pika.arm.isFinished()) {
                Pika.outtakeSlides.freeMove();
            }
            else {
                if (slidePositionMode)
                    Pika.outtakeSlides.update();
            }

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
            telemetry.addData("SlidePositionMode: ", slidePositionMode);
            telemetry.addData("\nArm: \n", Pika.arm.getTelemetry());
            telemetry.addData("\nMiniArmPitchPos: ", Pika.claw.armPitch.getPosition());
            telemetry.addData("\nSlides: \n", Pika.outtakeSlides.getTelemetry());
            telemetry.addData("\nClawOpen: ", clawOpen);
            telemetry.addData("\nX: ", Pika.localizer.getX());
            telemetry.addData("\nY: ", Pika.localizer.getY());
            telemetry.addData("Tx: ", Pika.limelight.tX);
            telemetry.addData("Ty: ", Pika.limelight.tY);
            telemetry.addData("MAX1: ", Pika.claw.armPitch.getPosition());
            telemetry.addData("MAX2: ", Pika.claw.armPitchSupplement.getPosition());
            telemetry.addData("Tx: ", Pika.limelight.tX);
            telemetry.addData("Ty: ", Pika.limelight.tY);
            telemetry.update();
        }
        // 14480
        // 9587

    }

}
