package org.firstinspires.ftc.teamcode.component;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.Pika;

public class OuttakeSlides {

    private DcMotorEx slide1;
    private DcMotorEx slide2;
    public double slide1Power;
    public double slide2Power;
    public static double holdPower = 0;
    public double error;
    public static double slideK = -0.0005;
    private double power = 0.5;
    public double pw;
    public double ERROR = 2500;
    public int increment = 100;

    private final double zeroPwr = 0;

    public int holdPos;
    private double stallCurrent = 5.9;
    public double P = 0.00017;
    public double I = 0.000008;
    public double D = 0;
    public static int retractAmount = 7600;
    private PIDController slideController = new PIDController(P, I,D); //0.006
    private PIDController sampleSlideController = new PIDController(P, I, D);
    private final double searchForSamplePower = 0.3;
    public static double feedForward = 0.1;
    private final int holdChangeConstant = 5000;

    public int targetPos;
    public boolean holdSlides;


    public enum TurnValue {
        RETRACTED(0),
        TEST(2000),
        PREPARE_OUTTAKE(5000),
        BUCKET2(43000),
        HANG(24000), //880
        HANG_RETRACT(10000),
        MAX_EXTENSION_UP(44000),
        MAX_EXTENSION_DOWN(25000);

        int ticks;

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
        holdSlides = false;
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
        holdPos = targetPos;
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setTargetPosition(targetPos);
    }
    public int getCurrentPosition() {return slide1.getCurrentPosition();}
    public double getTargetPosition(){
        return slide1.getTargetPosition();
    }

    public void resetPID() {
        slideController.reset();
    }

    public void update() {
        if (holdSlides)
            setTargetPosition(getCurrentPosition());

        error = getTargetPosition() - getCurrentPosition();
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
        if ((Pika.arm.getTargetPosition() == Arm.ArmPos.OUTTAKE.getPosition() &&
                slide1.getCurrentPosition()<TurnValue.MAX_EXTENSION_UP.ticks) ||
                (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition() &&
                slide1.getCurrentPosition() < TurnValue.MAX_EXTENSION_DOWN.ticks)) {
            slide1.setPower(-power);
            slide2.setPower(power);
            holdPos = Math.min(slide1.getCurrentPosition()+ holdChangeConstant,
                    TurnValue.MAX_EXTENSION_UP.getTicks());
        }
        else {
            slide1.setPower(0);
            slide2.setPower(0);
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
//        if (Pika.arm.getTargetPosition() == Arm.ArmPos.INTAKE.getPosition()) {
//            ERROR = 100;
//        }
//        else {
//            ERROR = 1000;
//        }
        return Math.abs(slide1.getCurrentPosition()- slide1.getTargetPosition())<=ERROR;
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
                "\nSlide1 Pw:" + slide1.getPower() +
                "\nSlide2 Pw: " + slide2.getPower() +
                "\nSlides Finished: " + isFinished();
    }

    public double totalCurrent() {
        return slide1.getCurrent(CurrentUnit.AMPS) + slide2.getCurrent(CurrentUnit.AMPS);
    }
}