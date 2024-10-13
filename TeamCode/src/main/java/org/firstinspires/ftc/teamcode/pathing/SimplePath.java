package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;



public class SimplePath {

    Point[] waypoints;

    double pow;
    double pow2;
    double pow3;
    double pow4;

    double heading;

    private Point[] curvePoints;
    private Point[] curveDerivatives;
    private double[] curveHeadings;

    static double tIncrement = MotionPlanner.tIncrement;


    public SimplePath(double heading, Point... waypoints) {
        this.waypoints = waypoints;
        this.heading = normalizeDegrees(heading);
    }

    public SimplePath(){}


    public SimplePath(Point... waypoints) {
        this(0, waypoints);
    }

    public Point[] getWaypoints() {
        return waypoints;
    }

}
