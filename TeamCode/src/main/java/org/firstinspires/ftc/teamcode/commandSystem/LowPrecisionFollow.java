package org.firstinspires.ftc.teamcode.commandSystem;

import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;
import org.firstinspires.ftc.teamcode.pathing.Path;

public class LowPrecisionFollow extends Command {
    MotionPlannerEdit mp;
    Path traj;

    public LowPrecisionFollow(MotionPlannerEdit mp, Path traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.resume();
        mp.startTrajectory(traj);
    }

    @Override
    public void update() {
        mp.update();
    }

    @Override
    public boolean isFinished() {
        return mp.isFinished();
    }

    @Override
    public void stop() {}
}

