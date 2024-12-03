package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public double clawPos;
    public double maxAPos;
    public double maxBPos;
    public double prevSampleAngle;
    public final double diffThreshold = 10;
    public double orientation;
    public double pivotPos;
    public double pitchPos;
    public double servoAPos;
    public double servoBPos;
    public Servo pivotPitchA;
    public Servo pivotPitchB;//.8244
    public Servo armPitch;
    public Servo armPitchSupplement;

    Servo claw;
    public final double maxLowerBound = 1;

    public enum PitchPosition {
        GRAB(0.6617, 0.2851), PLACE(0.9467, 0.0);
        private final double aPos;
        private final double bPos;
        PitchPosition(double aPos, double bPos) {
            this.aPos = aPos;
            this.bPos = bPos;
        }

        public double[] getPosition() {

            return new double[] {aPos, bPos};
        }
    }

    public enum ArmPitch {
        DOWN(0.255), UP(0.255), RETRACT(0), GRAB(0.33), DEPOSIT(0.16);

        private final double value;

        ArmPitch(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }


    }
    public enum ClawPosition {
        OPEN(0.6845), CLOSE(0.2755);
        private final double value;

        ClawPosition(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }

    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        pivotPitchA = hwMap.get(Servo.class, "pitchA");
        pivotPitchB = hwMap.get(Servo.class, "pitchB");
        armPitch = hwMap.get(Servo.class, "clawPitch");
        orientation = 90;
        setPivotOrientation(orientation);
        setClaw(ClawPosition.CLOSE.value);
        setArmPitch(ArmPitch.RETRACT.getPosition());
    }
    public void setClaw(double pos) {
        clawPos = pos;
        claw.setPosition(pos);
    }

    public double orientationToPos(double angle) {
        return (angle / 180) * (0.676-0.2709) + 0.2709;

    }
    public void setPivot(double pos) {
        pivotPos = pos;
        double currentPos = (pivotPitchA.getPosition() + pivotPitchB.getPosition())/2;
        double newPosA = pivotPitchA.getPosition() + (pivotPos-currentPos);
        double newPosB = pivotPitchB.getPosition() + (pivotPos-currentPos);

        if (newPosA < 1 && newPosB < 1 && newPosA > 0 && newPosB > 0) {
            servoAPos = newPosA;
            servoBPos = newPosB;
            pivotPitchA.setPosition(servoAPos);
            pivotPitchB.setPosition(servoBPos);
        }
    }
    public void setPitch(double posA, double posB) {
        pivotPitchA.setPosition(posA);
        pivotPitchB.setPosition(posB);
    }

    public void setArmPitch(double pos) {
        pitchPos = pos;
        maxAPos = pos;
        maxBPos = 1-pos;
        armPitch.setPosition(pos);

    }
    public void upPitchPosition(double val) {
        if (servoAPos + val < maxLowerBound && servoBPos - val > 0) {
            servoAPos = pivotPitchA.getPosition() + val;
            servoBPos = pivotPitchB.getPosition() - val;
        }
        pivotPitchA.setPosition(servoAPos);
        pivotPitchB.setPosition(servoBPos);
    }

    public void downPitchPosition(double val) {
        if (servoAPos - val > 0 && servoBPos + val < 1) {
            servoAPos = pivotPitchA.getPosition() - val;
            servoBPos = pivotPitchB.getPosition() + val;
        }

        pivotPitchA.setPosition(servoAPos);
        pivotPitchB.setPosition(servoBPos);
    }


    public void setPivotOrientation(double orientation) {
        this.orientation = orientation;
        setPivot(orientationToPos(orientation));
    }



}
