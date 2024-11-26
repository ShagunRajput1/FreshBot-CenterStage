package org.firstinspires.ftc.teamcode.pathing;

public interface Path {
    public Point[] getCurvePoints();
    public Point[] getCurveDerivatives();
    public double[] getCurveHeadings();
    public double approximateLength();

    public Point getEndPoint();

}
