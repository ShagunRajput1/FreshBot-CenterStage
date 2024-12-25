package org.firstinspires.ftc.teamcode.component.localizer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

public class Localizer {

    private Encoder rightEncoder;
    private Encoder frontEncoder;
    private Encoder leftEncoder;
    private List<LynxModule> allHubs;

    private Imu imu;

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.629921; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 4.125856061990338; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = 4.875;//0.413904; // in; offset of the lateral wheel
    //public static double FORWARD_OFFSET = -1.1811; // in; offset of the lateral wheel
// 1.79 37.864
    private double x;
    private double y;
    private double xPrev;
    private double yPrev;


    private double lastHeading;
    private double lastX;
    private double lastY;

    private double rawX;
    private double rawY;
    private double heading; // radians

    private double r0;
    private double r1;

    private double relX;
    private double relY;

    public static double X_MULTIPLIER = 1.01235993051; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.00928330372; // Multiplier in the Y direction
    private ElapsedTime time;
    private double elapsedTime;
    private double velocity;
    private double theta;
    private double avgVel;
    private double lastVelocity;
    private double acceleration;

    // Making arrays of velocities and acc to get a non-noisy average of last 10 values
    public double[] velocities;
    public double[] accelerations;

    public final int length = 10;
    private int index = 0;

    public Localizer(LinearOpMode opMode, HardwareMap hardwareMap, boolean twoWheel){
        velocities = new double[length];
        accelerations = new double[length];
        time = new ElapsedTime();
        imu = new Imu(hardwareMap);
        imu.initImuThread(opMode);

        reset();
    }

    public Localizer(LinearOpMode opMode, HardwareMap hardwareMap){

//        imu = new Imu(hardwareMap);
//        imu.initImuThread(opMode);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        velocities = new double[length];
        accelerations = new double[length];
        time = new ElapsedTime();


        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lb"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);


        rightEncoder.reset();
        frontEncoder.reset();
        leftEncoder.reset();

        reset();
    }

    public void reset(){
        x = 0;
        y = 0;

        lastHeading = 0;
        lastX = 0;
        lastY = 0;

    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public void update(){
        elapsedTime = time.seconds();
        time.reset();
        calculateRawValues();

        if(heading -lastHeading == 0){
            relX = (rawX -lastX);
            relY = (rawY -lastY);
        }else{
            r0 = (rawX -lastX) / (heading - lastHeading);
            r1 = (rawY -lastY) / (heading - lastHeading);

            relX = r0 * Math.sin(heading -lastHeading) - r1 * (1 - Math.cos(heading -lastHeading));
            relY = r1 * Math.sin(heading -lastHeading) + r0 * (1 - Math.cos(heading -lastHeading));
        }

        x += relX * Math.cos(heading) - relY * Math.sin(heading);
        y += relY * Math.cos(heading) + relX * Math.sin(heading);


//        velocity = Math.sqrt(
//                Math.pow((x-xPrev)/elapsedTime, 2) + Math.pow((y-yPrev)/elapsedTime, 2)
//        );
//        theta = Math.toDegrees(Math.atan2(y-yPrev, x-xPrev)) - getHeadingImu();

        // Average velocity - running average of 10 values
//        if (index<length) {
//            velocities[index] = getRawVelocity();
//            index++;
//        }
//        else {
//            index = 0;
//        }
//        double sum = 0;
//        for (int i =0; i<length;i++) {
//            sum+=velocities[i];
//        }
//        lastVelocity = avgVel;
//        avgVel = sum/length;
//
//        // Getting average acceleration - running sum of 10
//        double rawAcc = (avgVel-lastVelocity)/elapsedTime;
//        if (index<length) {
//            accelerations[index] = rawAcc;
//            index++;
//        }
//        else {
//            index = 0;
//        }
//        // Re-using the previous sum variable
//        sum = 0;
//        for (int i =0; i<length;i++) {
//            sum+=accelerations[i];
//        }
//        acceleration = sum/length;

//        xPrev = x;
//        yPrev = y;
        lastX = rawX;
        lastY = rawY;

        lastHeading = heading;
    }

    public void calculateRawValues(){
        double rawX = (getRightEncoderPosition() + getLeftEncoderPosition())/2.0;
        double heading = (getRightEncoderPosition() - getLeftEncoderPosition())/(LATERAL_DISTANCE);
        double rawY = getFwdEncoderPosition() - (FORWARD_OFFSET * heading);

        setRawValues(rawX, rawY, heading);
    }

    public double getLastHeading(){
        return lastHeading;
    }

    public void setRawValues(double x, double y, double head){
        rawX = x;
        rawY = y;
        heading = head;
    }


    public double getHeading(Angle angle) {
        if (angle == Angle.RADIANS) {
            return getHeading();
        } else {
            return Math.toDegrees(getHeading());

        }
    }


    public double getHeading(){
        return heading;
    }

    public double getRawX(){
        return rawX;
    }

    public double getRawY(){
        return rawY;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getRightEncoderPosition(){
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    public double getLeftEncoderPosition(){
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    public double getFwdEncoderPosition(){
        return encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }

    public void initImu(){
        imu.initIMU();
    }

    public enum Angle{
        RADIANS,
        DEGREES,
    }

    public double getHeadingImu() {
        return imu.getHeading();
    }
    public double getTheta() {
        return theta;
    }

    public double getAngularVelocityImu() {
        return imu.getAngularVelocity();
    }

    public String getTelemetry() {
        return "First heading: " + getHeading(Angle.DEGREES) +
                "\nX: " + getX() +
                "\nY: " + getY();
    }

    public double getRawVelocity() {
        return velocity;
    }

    public double getAvgVelocity() {
        return avgVel;
    }

    public double getAcc() {
        return acceleration;
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setHeading(double heading) {
        this.heading = heading;
    }

}