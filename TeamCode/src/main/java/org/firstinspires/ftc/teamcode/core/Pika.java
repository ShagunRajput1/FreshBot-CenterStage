package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Camlight;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Outtake;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

public class Pika {
    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static double movementPower;
    public static Intake intake;
    public static OuttakeSlides outtakeSlides;
    public static Outtake outtake;
    public static Claw claw;
    public static Arm arm;
    public static Camlight limelight;
    public static Localizer localizer;

    public static void init(HardwareMap hwMap, LinearOpMode opMode, boolean teleOp) {
        movementPower = 0.8;
        hardwareMap = hwMap;
        arm = new Arm();
        arm.init(hardwareMap, teleOp);
        limelight = new Camlight();
        limelight.init(hardwareMap);
        localizer = new Localizer(opMode, hardwareMap);
        drivetrain = new MecanumDrive(hwMap);
        claw = new Claw();
        claw.init(hardwareMap);
        outtakeSlides = new OuttakeSlides();
        outtakeSlides.init(hardwareMap, teleOp);

        claw.init(hardwareMap);

    }
}
