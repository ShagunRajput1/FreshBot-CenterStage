package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.util.TweakedPID;

public class SimpleFollower {
    ElapsedTime timer;
    public static double KStaticX = 0.08;
    public static double KStaticY = 0.14;
    public static double KStaticTurn = 0.08;
    public static double KStaticTurnEnd = 0.285;
    public static TweakedPID translationalControlX = new TweakedPID(0.02, 0.0015, 0.00065);
    public static TweakedPID translationalControlY = new TweakedPID(0.03, 0, 0);
    public static TweakedPID headingControl = new TweakedPID(0.01, 0.00, 0);
    public static PIDController headingControlEnd = new PIDController(0.01, 0.00, 0);
    int index = 0;
    double curX, targetX, xError, xPower;
    double curY, targetY, yError, yPower;
    double targetHeading, currentHeading;
    double finalMagnitude;
    double translationalError;
    double PIDMagnitude, theta, driveTurn;
    double translationalPIDPower;
    double headingError;
    final double headingErrorThreshold = 1.5;
    final double translationalErrorThreshold= 1;
    double brakeHeading;
    boolean end = false;
    Drivetrain drive;
    Localizer localizer;
    HardwareMap hwMap;
    SimplePath path;

    public SimpleFollower(Drivetrain drive, Localizer localizer, HardwareMap hwMap) {
        translationalControlY.setIntegrationBounds(-10000000, 10000000);
        translationalControlX.setIntegrationBounds(-10000000, 10000000);
        headingControlEnd.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);

        this.drive = drive;
        this.localizer = localizer;
        this.hwMap = hwMap;
    }

    public void startTrajectory(SimplePath path) {
        end = false;
        this.path = path;
        reset();

    }

    public void reset(){
        timer = new ElapsedTime();
        index = 0;
        translationalControlX.reset();
        translationalControlY.reset();
        headingControl.reset();
        headingControlEnd.reset();
    }

    public void follow() {
        getACValues();
        while (index<path.waypoints.length-1) {
            if (distance(path.waypoints[index+1],  new Point(curX, curY)) <
                    distance(path.waypoints[index], new Point(curX, curY))) {
                index++;
            }
            else
                break;
        }
        if (index == path.waypoints.length-1 && distance(new Point(curX, curY), path.waypoints[index]) <= 10) {
            end = true;
        }
        targetX = path.waypoints[index].x;
        targetY = path.waypoints[index].y;
        targetHeading = path.waypoints[index].heading;

        // Get translational error
        xError = targetX - curX;
        yError = targetY - curY;
        translationalError = Math.hypot(xError, yError);
        headingError = getHeadingError();

        // Rotate x and y error based on current heading
        theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);
        xError = Math.cos(Math.toRadians(theta))*translationalError;
        yError = Math.sin(Math.toRadians(theta))*translationalError;

        // Set Powers
        if (Math.abs(xError)>translationalErrorThreshold) {
            xPower = translationalControlX.calculate(0, xError);
            xPower = xPower + Math.signum(xPower)* KStaticX;
        }
        else {
            xPower = 0;
        }
        if (Math.abs(yError)>translationalErrorThreshold) {
            yPower = translationalControlY.calculate(0, yError);
            yPower = yPower + Math.signum(yPower)* KStaticY;
        }
        else {
            yPower = 0;
        }
        PIDMagnitude = Math.hypot(xPower, yPower);
        if (Math.abs(headingError) > headingErrorThreshold) {
            driveTurn = headingControl.calculate(0, headingError);
            driveTurn = driveTurn + Math.signum(driveTurn)* KStaticTurn;
        }
        else {
            driveTurn = 0;
        }
        if (!isFinished()) {
            if (end)
                drive.drive(PIDMagnitude, theta, driveTurn, 0.85);
            else
                drive.driveMax(PIDMagnitude, theta, driveTurn, 0.85);


        }
        else {
            drive.drive(0,0,0,0);
            if (index<path.waypoints.length-1)
                index++;
        }




    }

    private void getACValues() {
        curX = localizer.getX();
        curY = localizer.getY();
        currentHeading = localizer.getHeading(Localizer.Angle.DEGREES);
    }
    private double distance(Point p1, Point p2){
        return Math.hypot(p1.getX()-p2.getX(), p1.getY()-p2.getY());
    }
    public double getHeadingError(){
        double headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }
    private boolean reachedHeadingTarget() {
        return (Math.abs(headingError) <= headingErrorThreshold);
    }
    private boolean isFinished() {
        return (reachedHeadingTarget() && reachedTranslationalTarget());
    }
    private boolean reachedTranslationalTarget() {
        return (Math.abs(Math.hypot(xError, yError))<=translationalErrorThreshold);
    }

    public String getTelemetry() {
        return  "X: " + curX +
                "\nY: " + curY +
                "\nHeading: " + currentHeading +
                "\nTargetX: " + targetX +
                "\nTargetY: " + targetY +
                "\nTargetHeading: " + targetHeading +
                "\nxError: " + xError +
                "\nyError: " + yError +
                "\nxPower: " + xPower +
                "\nyPower: " + yPower +
                "\nMagnitude: " + PIDMagnitude +
                "\nDriveTurn: " + driveTurn +
                "\nTheta: " + theta +
                "\nTranslationalError" + translationalError +
                "\nheadingError: " + headingError +
                "\nPathLength: " + path.waypoints.length +
                "\nisFinished: " + isFinished() +
                "\nWayPoints: " + getWayPointsString() +
                "\nEnd: " + end;
    }

    public String getWayPointsString() {
        String wayPoints = "";
        for (int index = 0; index<path.waypoints.length; index++) {
            wayPoints = wayPoints.concat("" + path.waypoints[index].x + ", " + path.waypoints[index].y + "\n");
        }
        return wayPoints;
    }


}
