package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Camlight;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Outtake;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.localizer.Localizer;
import org.firstinspires.ftc.teamcode.component.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

public class Pika {
    static double xOffset = 0;
    static double yOffset = 0;

    public static HardwareMap hardwareMap;
    public static MecanumDrive drivetrain;
    public static double movementPower;
    public static Intake intake;
    public static OuttakeSlides outtakeSlides;
    public static Outtake outtake;
    public static FinalClaw newClaw;
    public static Arm arm;
    public static Camlight limelight;
    public static Localizer localizer;
    public static Pinpoint pinpoint;

    public static void init(HardwareMap hwMap, LinearOpMode opMode, boolean teleOp) {
//        pinpoint = hardwareMap.get(Pinpoint.class, "odo");
//        pinpoint.setOffsets(xOffset, yOffset);
//        pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.resetPosAndIMU();

        movementPower = 0.8;
        hardwareMap = hwMap;
        arm = new Arm();
        limelight = new Camlight();
        limelight.init(hardwareMap);
        localizer = new Localizer(opMode, hardwareMap);
        drivetrain = new MecanumDrive(hwMap);
        newClaw = new FinalClaw();
        newClaw.init(hardwareMap);
        outtakeSlides = new OuttakeSlides();
        outtakeSlides.init(hardwareMap, teleOp);
        arm.init(hardwareMap, teleOp);
        if (teleOp)
            newClaw.setMiniPitch(FinalClaw.MiniPitch.BEFORE_GRAB.getPosition());

    }

    public static double getVoltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
}
