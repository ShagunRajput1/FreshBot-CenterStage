package org.firstinspires.ftc.teamcode.pathing;

public class RobotPose {
    double x;
    double y;
    double heading;
    public RobotPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }

}
