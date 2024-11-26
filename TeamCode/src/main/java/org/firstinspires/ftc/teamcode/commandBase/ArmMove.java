package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.core.Pika;

public class ArmMove extends Command {
    int pos;

    public ArmMove(int pos) {
        this.pos = pos;
    }

    @Override
    public void init() {
        Pika.arm.setTargetPosition(pos);
    }


    @Override
    public void update() {
        Pika.outtakeSlides.freeMove();
    }

    @Override
    public boolean isFinished() {
        return Pika.arm.isFinished();
    }
}
