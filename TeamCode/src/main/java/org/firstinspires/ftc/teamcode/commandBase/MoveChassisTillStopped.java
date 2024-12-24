package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.core.Pika;

public class MoveChassisTillStopped extends Command {
    double currentThreshold = 8;
    @Override
    public void init() {
        Pika.drivetrain.drive(1, 0, 0, 0.75, Pika.getVoltage());
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Pika.drivetrain.totalCurrent() >= currentThreshold;
    }

    @Override
    public void stop() {

    }
}
