package org.firstinspires.ftc.teamcode.component;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.tuningTeleop.AlignWithSample;

public class OuttakeSlides {

    private DcMotorEx slide1;
    private DcMotorEx slide2;
    public double slide1Power;
    public double slide2Power;
    public static double holdPower = 0;
    public double error;
    public static double slideK = -0.0005;
    private double power = 0.4;
    public double pw;
    public double ERROR = 1800;
    public int increment = 100;

    private final double zeroPwr = 0;

    public int holdPos;
    private double stallCurrent = 5.9;
    public double P = 0.0003;
    public double I = 0.0001;
    public double D = 0;
    public static int retractAmount = 7600;
    public static int extendAmountIntake = 3050;
    private final PIDController slideController = new PIDController(P, I,D); //0.006
    private final PIDController sampleSlideController = new PIDController(0.008, 0.0012, 0);
    private final double searchForSamplePower = 0.325;
    public static double feedForward = 0.1;
    private final int holdChangeConstant = 5000;
    public int currentPos;

    public int targetPos;
    public boolean toUpdate;


    public enum TurnValue {
        RETRACTED(500),
        HANG_RETRACT(6000),
        BUCKET2(40500),
        BUCKET2_TELEOP(36500),
        HANG(24000), //880
        MAX_EXTENSION_UP(42000),
        MAX_EXTENSION_DOWN(18500),
        SPEC_DEPOSIT(15000),
        SPEC_PREP(6200);

        final int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public void init(HardwareMap hwMap, boolean telOp) {
        slide1 = hwMap.get(DcMotorEx.class, "slide1");
        slide2 = hwMap.get(DcMotorEx.class, "slide2");
        slide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideController.setIntegrationBounds(-10000000, 10000000);
        sampleSlideController.setIntegrationBounds(-10000000, 10000000);
        if (!telOp) {
            slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide1.setTargetPosition(0);
        }
        toUpdate = true;
        targetPos = 0;

        stallCurrent = hwMap.voltageSensor.iterator().next().getVoltage()/2.2;
    }

    public void resetEncoder() {
        slide1.setPower(0);
        slide2.setPower(0);
        slide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideController.reset();
    }


    public void setTargetPosition(int targetPos){
        this.targetPos = targetPos;
        holdPos = targetPos;
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setTargetPosition(targetPos);
    }
    public int getCurrentPosition() {
        currentPos = slide1.getCurrentPosition();
        return currentPos;
    }
    public double getTargetPosition(){
        return targetPos;
    }

    public void resetPID() {
        slideController.reset();
    }

    public void update() {
        int currentPos = getCurrentPosition();
        if (!toUpdate)
            return;
        error = getTargetPosition() - currentPos;
        if (targetPos == TurnValue.RETRACTED.getTicks() && currentPos < TurnValue.RETRACTED.getTicks() && slide1Power>0) {
            // Prevents strain. Target is retracted. CurrentPos is below Target and Slide is being pulled back. Happens sometimes idk why
            slide1.setPower(0);
            slide2.setPower(0);
            return;
        }

        pw = Range.clip(slideController.calculate(0, error), -1, 1);
        slide1Power = -pw;
        slide2Power = pw;
        if (Math.abs(error) < ERROR && Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition())
            stopSlides();
        else {
            slide1.setPower(-pw);
            slide2.setPower(pw);
        }
    }

    public void goUp() {
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (((Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() ||
                Pika.arm.getTargetPosition() == Arm.ArmPos.PREP_HANG.getPosition()) &&
                slide1.getCurrentPosition()<TurnValue.MAX_EXTENSION_UP.ticks) ||
                (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
                slide1.getCurrentPosition() < TurnValue.MAX_EXTENSION_DOWN.ticks)) {
            slide1.setPower(-power);
            slide2.setPower(power);
            holdPos = Math.min(slide1.getCurrentPosition()+ holdChangeConstant,
                    TurnValue.MAX_EXTENSION_UP.getTicks());
        }
        else {
            // Don't wanna un-power slides when extended up
            if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
                slide1.setPower(0);
                slide2.setPower(0);
            }
        }

    }

    public void goDown() {
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (slide1.getCurrentPosition() > TurnValue.RETRACTED.ticks) {
            slide1.setPower(power);
            slide2.setPower(-power);
            holdPos = Math.max(slide1.getCurrentPosition() - holdChangeConstant, TurnValue.RETRACTED.getTicks());
        }
        else {
            slide1.setPower(0);
            slide2.setPower(0);
        }
    }
    public void retractToIntake() {
        setTargetPosition(slide1.getCurrentPosition() - retractAmount);
    }
    public void holdSlides() {
        setTargetPosition(holdPos);
    }
    public boolean isBusy(){
        return slide1.isBusy();
    }

    public boolean isFinished(){
        return Math.abs(currentPos - targetPos)<=ERROR;
    }

    public boolean isStalling(){
        return slide1.getCurrent(CurrentUnit.AMPS)>stallCurrent;
    }

    public void stopSlides(){
        // value added to prevent sliding down
//        slide1.setPower(Range.clip(zeroPwr + slide1.getCurrentPosition() * 0.0001, -1, 1));
//        slide2.setPower(Range.clip(zeroPwr + slide1.getCurrentPosition() * 0.0001, -1, 1));
        slide1.setPower(0);
        slide2.setPower(0);
    }

    public void setPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        slideController.setPID(P, I, D);
    }

    public void freeMove() {
        slide1Power = 0;
        slide2Power = 0;
        slide1.setPower(slide1Power);
        slide2.setPower(slide2Power);
    }

    public void alignWithSample(double error) {
        pw = Range.clip(sampleSlideController.calculate(0, error), -1, 1);
        pw = pw + Math.signum(pw)*AlignWithSample.slideKStatic;
        slide1Power = -pw;
        slide2Power = pw;
        slide1.setPower(-pw);
        slide2.setPower(pw);
        targetPos = slide1.getCurrentPosition();
    }
    public void sampleSetPID(double P, double I, double D) {
        sampleSlideController.setPID(P, I, D);
    }

    public void extendForSample() {
        this.power = searchForSamplePower;
        goUp();
    }

    public void setPower(double pw) {
        power = pw;
    }
    public String getTelemetry() {
        return  "TargetPos: " + getTargetPosition() +
                "\nCurrentPos: " + getCurrentPosition() +
                "\nError:" + error +
                "\nSlide1 Pw:" + slide1.getPower() +
                "\nSlide2 Pw: " + slide2.getPower() +
                "\nSlides Finished: " + isFinished() +
                "\ntoUpdate: " + toUpdate;
    }

    public double totalCurrent() {
        return slide1.getCurrent(CurrentUnit.AMPS) + slide2.getCurrent(CurrentUnit.AMPS);
    }

    public void resume() {
        toUpdate = true;
    }

    public void pause() {
        toUpdate = false;
    }
}