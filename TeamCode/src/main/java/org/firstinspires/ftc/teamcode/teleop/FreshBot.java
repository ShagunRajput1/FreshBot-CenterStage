package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
@Config
@TeleOp
public class FreshBot extends LinearOpMode {
    public static double P=0, I=0, D=0;
    ElapsedTime headingTimer = new ElapsedTime();

    // add extends LinearOpMode to inherit object Linear OpMode
    public void runOpMode() throws InterruptedException {  // throws error if something is wrong
        Drivetrain drivetrain = new MecanumDrive(hardwareMap);  // creates new instance of the object Drivetrain; includes hardware map
        Imu imu = new Imu(hardwareMap);
        PIDController headingControl = new PIDController(P, I, D);
        double targetheading = 0;
        double turnpower = 0;
        boolean holdHeading = true;

        waitForStart(); // initialising code will be entered before; if nothing then have to start to start code
        while(opModeIsActive()){

            double driveTurn = Math.pow(-gamepad1.right_stick_x, 1); // adds game pad controls to
            double driveY = -Math.pow(-gamepad1.left_stick_x, 1);
            double driveX = -Math.pow(-gamepad1.left_stick_y, 1);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.toDegrees(Math.atan2(driveY, driveX));
            theta = theta - imu.getHeading();
            headingControl.setPID(P,I,D);

            if(driveTurn == 0){
                if (headingTimer.seconds()>1){
                double currentHeading = imu.getHeading();
                double error = targetheading - currentHeading;
                turnpower = headingControl.calculate(0, error);
                    if(Math.abs(error)<2){
                         turnpower = 0;
                 }
                driveTurn = turnpower;
                }
                else{
                    targetheading = imu.getHeading();
                }
            }
            else{
               headingTimer.reset();
                targetheading = imu.getHeading();

            }

            drivetrain.drive(magnitude, theta, driveTurn, 0.8);


            telemetry.addData("heading: ", imu.getHeading());
            telemetry.addData("turnPower: ", turnpower);
            telemetry.addData("targetHeading: ", targetheading);
            telemetry.addData("driveTurn: ", driveTurn);
            telemetry.update();

        }
    }

}