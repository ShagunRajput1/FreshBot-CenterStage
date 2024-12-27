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
        OPEN(0.058), CLOSE(0.295), LITTLE_OPEN(0.1205);
        private final double value;

        ClawPosition(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }
    public enum ArmPitch {
        UP(0.6865), RETRACT(0.0975), GRAB(0.718), DEPOSIT(0.5475), APRIL(0.343),
        BEFORE_GRAB(0.66), SPEC_DEPOSIT(0.261), SPEC_GRAB(0.4725);

        private final double value;

        ArmPitch(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }

    public enum MiniPitch {
        RETRACT(0.793), GRAB(0.512), DEPOSIT(1), SPEC_DEPOSIT(0.71),
        SPEC_GRAB(0.379);

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
        return (angle / 180) * (0.904-0) + 0;

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
