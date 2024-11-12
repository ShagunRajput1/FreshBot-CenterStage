package org.firstinspires.ftc.teamcode.pathing;

public class PathPoint extends Point {
    double x;
    double y;
    double heading;

    public PathPoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public PathPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

        public double getHeading() {
        return heading;
    }
}
