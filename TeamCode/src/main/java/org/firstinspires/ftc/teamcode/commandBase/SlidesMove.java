package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.core.Pika;

public class SlidesMove extends Command {
    int ticks;

    public SlidesMove(int ticks) {
        this.ticks = ticks;
    }

    @Override
    public void init() {
        Pika.outtakeSlides.setTargetPosition(ticks);
    }


    @Override
    public void update() {
        Pika.outtakeSlides.setTargetPosition(ticks);
    }

    @Override
    public boolean isFinished() {
        return Pika.outtakeSlides.isFinished();
    }

    @Override
    public void stop() {}
}
