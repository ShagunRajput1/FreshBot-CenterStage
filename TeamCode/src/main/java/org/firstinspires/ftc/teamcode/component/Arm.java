package org.firstinspires.ftc.teamcode.component;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.TweakedPID;


public class Arm {
    DcMotorEx armMotor;
    double currentPosition;
    double error;
    int targetPosition;
    double power;
    public double P = 0.009;
    public double I = 0.0002;
    public double D = 0.0001;
    private final int ERROR = 50;
    private double movementPower = 1;
    TweakedPID armController = new TweakedPID(P, I, D);


    public enum ArmPos {
        INTAKE(0), OUTTAKE(1360), HANG(450), PREP_HANG(1650), STUPID_TAIL(1530),
        SPEC_DEPOSIT(1360), TELEOP_DEPOSIT(1435);
        private final int value;
        ArmPos(int val) {
            this.value = val;
        }
        public int getPosition() {
            return value;
        }
    }
    public void init(HardwareMap hwMap, boolean teleop) {
        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        if (!teleop)
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
        armController.setIntegrationBounds(-10000000, 10000000);
    }

    public void pitchUp() {
        if (targetPosition<=1560)
            setTargetPosition(targetPosition+4);
    }

    public void pitchDown() {
        if (targetPosition>=0)
            setTargetPosition(targetPosition-4);
    }
    public int getTargetPosition() {
        return targetPosition;
    }
    public void setTargetPosition(int target) {
        targetPosition = target;
    }

    public int getPosition() {
        return armMotor.getCurrentPosition();
    }

    public void update() {
        currentPosition = getPosition();
        double error = targetPosition-currentPosition;
        power = Range.clip(armController.calculate(0, error), -1, 1);
        if (Math.abs(power)>movementPower) {
            power = Math.signum(power)*movementPower;
        }
        armMotor.setPower(power);
    }

    public String getTelemetry() {
        return  "CurrentPos: " + armMotor.getCurrentPosition() +
                "\nTarget Pos: " + targetPosition +
                "\nPower: " + power +
                "\nFinished: " + (Math.abs(armMotor.getCurrentPosition()- targetPosition)<=ERROR);
    }

    public void setPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        armController.setPID(P, I, D);
    }

    public boolean isFinished(){
        return Math.abs(currentPosition- targetPosition)<=ERROR;
    }

    public double totalCurrent() {
        return armMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void setMaxMovementPower(double pw) {
        movementPower = pw;
    }
}
