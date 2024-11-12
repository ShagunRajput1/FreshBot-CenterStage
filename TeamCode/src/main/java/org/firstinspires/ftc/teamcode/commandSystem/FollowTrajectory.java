package org.firstinspires.ftc.teamcode.commandSystem;


import org.firstinspires.ftc.teamcode.pathing.Bezier;
import org.firstinspires.ftc.teamcode.pathing.MotionPlanner;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

public class FollowTrajectory extends Command {
    MotionPlannerEdit mp;
    Bezier traj;

    public FollowTrajectory(MotionPlannerEdit mp, Bezier traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.startTrajectory(traj);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mp.isFinished();
    }
}
