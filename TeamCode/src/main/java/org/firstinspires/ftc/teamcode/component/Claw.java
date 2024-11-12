package org.firstinspires.ftc.teamcode.component;

import android.app.VoiceInteractor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public double clawPos;
    public double prevSampleAngle;
    public final double diffThreshold = 10;
    public double pivotPos;
    public double servoAPos;
    public double servoBPos;
    Servo pivotPitchA;
    Servo pivotPitchB;
    Servo claw;

    public enum ClawPosition {
        OPEN(0), CLOSE(0.25);
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
        pivotPitchA = hwMap.get(Servo.class, "pivot");
        pivotPitchB = hwMap.get(Servo.class, "pivot");
        setClaw(ClawPosition.OPEN.value);
    }
    public void setClaw(double pos) {
        clawPos = pos;
        claw.setPosition(pos);
    }

    public double orientationToPos(double angle) {
        return (angle / 90) * 0.571;

    }
    public void setPivot(double pos) {
        pivotPos = pos;
        double currentPos = (pivotPitchA.getPosition() + pivotPitchB.getPosition())/2;
        servoAPos = pivotPitchA.getPosition() + (pivotPos-currentPos);
        servoBPos = pivotPitchB.getPosition() + (pivotPos-currentPos);
        pivotPitchA.setPosition(servoAPos);
        pivotPitchB.setPosition(servoBPos);
    }

    public void setPivotOrientation(double orientation) {
        setPivot(orientationToPos(orientation));
    }



}
