package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private Servo arm;

    public enum ArmPos {
        LOAD(1), DUMP(0.511);
        private final double pos;

        ArmPos(double val) {
            this.pos = val;
        }

        public double getPos() {
            return pos;
        }
    }
    private double armPos;
    private double dumpPos;

    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "arm");
        arm.setPosition(ArmPos.LOAD.pos);
        armPos = ArmPos.LOAD.getPos();
    }

    public void liftArm() {
        if (armPos > ArmPos.DUMP.getPos())
            armPos-=0.005;
        setArmPosition(armPos);
    }
    public void lowerArm() {
        armPos+=0.01;
        setArmPosition(armPos);
    }

    public void setArmPosition(double pos) {
        armPos = pos;
        arm.setPosition(pos);
    }

    public double getArmPosition() {return arm.getPosition();}

}
