package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FreshSlides {
     DcMotorEx Slides1;
     DcMotorEx Slides2;

     double slidePower;

    // constructor
    public FreshSlides(HardwareMap hwmap){
        Slides1 = hwmap.get(DcMotorEx.class, "Slide1");
        Slides2 = hwmap.get(DcMotorEx.class, "Slide2");
        Slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidePower = 1;
    }

    public void goUp(){
        Slides1.setPower(slidePower);
        Slides2.setPower(slidePower);
    }

    public void goDown(){
        Slides1.setPower(-slidePower);
        Slides2.setPower(-slidePower);
    }



}
