package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.core.Pika;

public class SlidesMoveRelative extends Command {
    int ticks;
    public int position;

    public SlidesMoveRelative(int ticks) {
        this.ticks = ticks;
    }

    @Override
    public void init() {
        int position = Pika.outtakeSlides.getCurrentPosition() + ticks;
        Pika.outtakeSlides.setTargetPosition(position);
    }


    @Override
    public void update() {
        Pika.outtakeSlides.setTargetPosition(position);
    }

    @Override
    public boolean isFinished() {
        return Pika.outtakeSlides.isFinished();
    }
}

