package org.firstinspires.ftc.teamcode.pathing;

import java.util.Arrays;
import java.util.List;

public class CatmullRom implements Path {
    Point[] controlPoints;
    double heading;
    boolean constantHeading;
    private Point[] curvePoints;
    private Point[] curveDerivatives;
    private double[] curveHeadings;
    private double segmentParts;
    private final double tIncrement = 0.05;

    public CatmullRom(double heading, boolean constantHeading, Point... waypoints) {
        this.heading = heading;
        this.constantHeading = constantHeading;
        controlPoints = new Point[waypoints.length+2];
        // Duplicate first and last element in the list
        controlPoints[0] = waypoints[0];
        System.arraycopy(waypoints, 1, controlPoints, 1, waypoints.length - 1);
        controlPoints[controlPoints.length-1] = waypoints[waypoints.length-1];

    }

    private void generateCurve() {
        int length = ((int) (1.0/tIncrement)) * controlPoints.length-3;
        curvePoints = new Point[length];
        curveDerivatives = new Point[length];
        curveHeadings = new double[length];
        int index = 0;
        for (int i =0; i<controlPoints.length-3; i++) {
            for (double t = 0; t <= 1; t += tIncrement) {  // Adjust increment for smoother curve
                // Calculate position on the spline
                Point p = getCatmullPoint(controlPoints[i], controlPoints[i + 1],
                        controlPoints[i + 2], controlPoints[i + 3], t);
                curvePoints[index] = p;

                // Calculate derivative (tangent) on the spline
                Point derivative = getCatmullDerivative(controlPoints[i], controlPoints[i + 1],
                        controlPoints[i + 2], controlPoints[i + 3], t);
                curveDerivatives[index] = derivative;
                if (constantHeading) {
                    curveHeadings[index] = heading;
                }
                else {
                    curveHeadings[index] = Math.toDegrees(Math.atan2(derivative.y, derivative.x));
                }
            }
        }
    }

    private Point getCatmullPoint(Point p0, Point p1, Point p2, Point p3, double t) {
        double x = 0.5 * ((2 * p1.x) +
                (-p0.x + p2.x) * t +
                (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * Math.pow(t, 2) +
                (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * Math.pow(t, 3));

        double y = 0.5 * ((2 * p1.y) +
                (-p0.y + p2.y) * t +
                (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * Math.pow(t, 2) +
                (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * Math.pow(t, 3));
        return new Point(x, y);
    }

    private Point getCatmullDerivative(Point p0, Point p1, Point p2, Point p3, double t) {
        double x = 0.5 * ((-p0.x + p2.x) +
                (2 * (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x)) * t +
                3 * (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * Math.pow(t, 2));
        double y = 0.5 * ((-p0.y + p2.y) +
                (2 * (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y)) * t +
                3 * (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * Math.pow(t, 2));
        return new Point(x, y);
    }

    public Point[] getCurvePoints() {
        return curvePoints;
    }

    public Point[] getCurveDerivatives() {
        return curveDerivatives;
    }


    public double[] getCurveHeadings() {
        return curveHeadings;
    }

    public double approximateLength() {
        double length = 0;
        for (int i = 0; i<curvePoints.length-1; i++) {
            length += distance(curvePoints[i], curvePoints[i+1]);
        }
        return length;
    }
    public Point getEndPoint() {
        return controlPoints[controlPoints.length-1];
    }


    private double distance(Point a, Point b) {
        return Math.hypot((b.x-a.x), (b.y-a.y));
    }
}
