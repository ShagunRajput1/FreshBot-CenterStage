package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private final double rollerPower = 1;
    private final double maxThreshold = 0.8078;

    private final double extendoTinyIncrement = 0.01;
    private static double extendoIncrement = -0.0025;


    private Servo extendo;
    private Servo pivot;
    private CRServo roller1;
    private CRServo roller2;

    private DistanceSensor distSensor;

    public enum ExtensionPosition {
        SUPER_RETRACTED(1), RETRACTED(0.9711), EXTENDED(0.824);
        private final double value;

        ExtensionPosition(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }

    public enum PivotPosition {
        Down(1), Up(0.366);
        private final double value;

        PivotPosition(double value) {
            this.value = value;
        }

        public double getPosition() {
            return value;
        }
    }

    public void init(HardwareMap hwMap, boolean auto) {
        roller1 = hwMap.get(CRServo.class, "roller1");
        roller2 = hwMap.get(CRServo.class, "roller2");
        extendo = hwMap.get(Servo.class, "extendo");
        pivot = hwMap.get(Servo.class, "pivot");
        distSensor = hwMap.get(DistanceSensor.class, "distSensor");
        extendo.setPosition(ExtensionPosition.SUPER_RETRACTED.getPosition());
        pivot.setPosition(PivotPosition.Up.getPosition());
    }



    public void setExtendoPostion(double targetPos) { // in ticks
        extendo.setPosition(targetPos);
    }

    public double getPosition() {return extendo.getPosition();}

    public void setPivotPosition(double targetPos) {pivot.setPosition(targetPos);}
    public double getPivotPosition() {return pivot.getPosition();}
    public void incrementPivot() {pivot.setPosition(pivot.getPosition() + extendoTinyIncrement);}
    public void decrementPivot() {pivot.setPosition(pivot.getPosition() - extendoTinyIncrement);}
    public void extend() {
        double pos = extendo.getPosition() + extendoIncrement;
        if (pos < maxThreshold)
            pos = maxThreshold;
        extendo.setPosition(pos);
    }

    public void retract() {
        extendo.setPosition(extendo.getPosition() - extendoIncrement);
    }

    public void rollerIn() {
        if (distSensor.getDistance(DistanceUnit.MM) > 10) {
            roller1.setPower(rollerPower);
            roller2.setPower(-rollerPower);
        }
        else {
            roller1.setPower(0);
            roller2.setPower(0);
        }
    }
    public void rollerOut() {
        roller1.setPower(-rollerPower);
        roller2.setPower(rollerPower);
    }

    public void rollerStop() {
        roller1.setPower(0);
        roller2.setPower(0);
    }

    public double getDistance() {
        return distSensor.getDistance(DistanceUnit.MM);
    }
//    public String getRGB() {
//        return "R: " + colorSensor.red() +
//                "\n G: " + colorSensor.green() +
//                "\n B: " + colorSensor.blue();
//    }

}
