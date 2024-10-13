package org.firstinspires.ftc.teamcode.component;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OuttakeSlides {

    private DcMotorEx slide1;
    private DcMotorEx slide2;
    public static double slideK = -0.0005;
    private double power = 0.5;
    public double pw;
    private final double ERROR = 67.5;
    public int increment = 100;

    private final double zeroPwr = 0;

    private double stallCurrent = 5.9;
    public static double P = 0.003;
    public static double I = 0.0007;
    public static double D = 0;
    private PIDController slideController = new PIDController(P, I,D); //0.006
    public static double feedForward = 0.1;


    public enum TurnValue {
        SUPER_RETRACTED(-100),
        RETRACTED(0),
        INTAKE(0),
        BUCKET(750), // 880
        BUCKET2(-800),
        CLIMB(1960); //880

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public void init(HardwareMap hwMap, boolean teleop) {
        slide1 = hwMap.get(DcMotorEx.class, "slide1");
        slide2 = hwMap.get(DcMotorEx.class, "slide2");
//        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        if (!teleop) {
//            slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        stallCurrent = hwMap.voltageSensor.iterator().next().getVoltage()/2.2;

//        setTargetPosition(0);
    }

    public void setTargetPosition(int targetPos){
        slide1.setTargetPosition(targetPos);
    }
    public int getCurrentPosition() {return slide1.getCurrentPosition();}
    public double getTargetPosition(){
        return slide1.getTargetPosition();
    }

    public void update(){
            pw = Range.clip(slideController.calculate(0, slide1.getTargetPosition() - slide1.getCurrentPosition()), -1, 1);
            // sets power and mode
//            pw = pw + feedForward*(slide1.getCurrentPosition());
            slide1.setPower(pw);
            slide2.setPower(pw);


    }

    public double getCurrent1(){
        return slide1.getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrent2(){
        return slide2.getCurrent(CurrentUnit.AMPS);
    }

    public void goUp() {
//        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pw = -1;
        slide1.setPower(-power);
        slide2.setPower(-power);
    }

    public void goDown() {
//        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pw = 1;
        slide1.setPower(power);
        slide2.setPower(power);
    }

    public int getTicks(){
        return slide1.getCurrentPosition();
    }

    public boolean isBusy(){
        return slide1.isBusy();
    }

    public boolean isFinished(){
        return Math.abs(slide1.getCurrentPosition()- slide1.getTargetPosition())<=ERROR;
    }

    public boolean isStalling(){
        return slide1.getCurrent(CurrentUnit.AMPS)>stallCurrent;
    }

    public void stopSlides(){
        // value added to prevent sliding down
        slide1.setPower(Range.clip(zeroPwr + slide1.getCurrentPosition() * 0.0001, -1, 1));
        slide2.setPower(Range.clip(zeroPwr + slide1.getCurrentPosition() * 0.0001, -1, 1));
//    slide1.setPower(0);
//    slide2.setPower(0);
    }

    public double getPW() {return pw;}

    public void holdSlide() {
        double pos = slide1.getCurrentPosition();
        pw = pos*slideK;
        slide1.setPower(pw);
        slide2.setPower(pw);
    }
}