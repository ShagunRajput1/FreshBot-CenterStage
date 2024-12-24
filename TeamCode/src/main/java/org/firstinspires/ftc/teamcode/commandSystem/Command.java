package org.firstinspires.ftc.teamcode.commandSystem;

/**
 * @author Snakuwul Joshi
 * **/

public abstract class Command {
    private String commandName;
    public void init(){
        return;
    }

    public abstract void update();

    public abstract boolean isFinished();
    public abstract void stop();
}