package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;



public class SimplePath {

    PathPoint[] waypoints;
    double heading;

    public SimplePath(double heading, PathPoint... waypoints) {
        this.waypoints = waypoints;
        this.heading = normalizeDegrees(heading);
    }

    public SimplePath(PathPoint... waypoints) {
        this(0, waypoints);
    }

    public PathPoint[] getWaypoints() {
        return waypoints;
    }

}
