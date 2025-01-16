package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FinalClaw {
    public Servo claw;
    public Servo pitchA;
    public Servo pitchB;
    public Servo miniPitch;
    public Servo pivot;

    public double clawPos, pitchPos, miniPitchPos, pivotPos;
    public double orientation;
    public enum ClawPosition {
        OPEN(0.2565), CLOSE(0.536), LITTLE_OPEN(0.1205);
        private final double value;

        ClawPosition(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }
    public enum ArmPitch {
        UP(0.684), RETRACT(0.1385), GRAB(0.7455), DEPOSIT(0.5675), APRIL(0.404),
        BEFORE_GRAB(0.6695), SPEC_DEPOSIT(0.686), SPEC_GRAB(0.735), AFTER_GRAB(0.65);

        private final double value;

        ArmPitch(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }

    public enum MiniPitch {
        RETRACT(0.578), GRAB(0.8245), DEPOSIT(0.4255), SPEC_DEPOSIT(0.336),
        SPEC_GRAB(0.7525), DETECT(0.8835), BEFORE_GRAB(0.838);

        private final double value;

        MiniPitch(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }

    }


    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        pitchA = hwMap.get(Servo.class, "pitchA");
        pitchB = hwMap.get(Servo.class, "pitchB");
        miniPitch = hwMap.get(Servo.class, "miniPitch");
        pivot = hwMap.get(Servo.class, "pivot");
        setPivotOrientation(180);
        claw.setPosition(ClawPosition.CLOSE.getPosition());
        pitchA.setPosition(ArmPitch.RETRACT.getPosition());
        pitchB.setPosition(1-ArmPitch.RETRACT.getPosition());
        miniPitch.setPosition(MiniPitch.RETRACT.getPosition());
    }

    public void setClaw(double pos) {
        clawPos = pos;
        claw.setPosition(pos);
    }

    public void setArmPitch(double pos) {
        pitchPos = pos;
        pitchA.setPosition(pos);
        pitchB.setPosition(1-pos);
    }

    public void setMiniPitch(double pos) {
        miniPitchPos = pos;
        miniPitch.setPosition(pos);
    }

    public void setPivot(double pos) {
        pivotPos = pos;
        pivot.setPosition(pos);
    }
    public static double orientationToPos(double angle) {
        return (angle / 180) * (0.57-0.019) + 0.019;

    }

    public void setPivotOrientation(double orientation) {
        this.orientation = orientation;
        setPivot(orientationToPos(orientation));
    }

    public String getTelemetry() {
        return "ClawPos: " + clawPos +
                "\nPivotPos: " + pivotPos +
                "\nOrientation: " + orientation +
                "\nPitchPos: " + pitchPos +
                "\nMiniPitchPos: " + miniPitchPos;
    }


}
