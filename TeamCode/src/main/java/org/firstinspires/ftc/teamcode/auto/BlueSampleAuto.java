package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.component.Camlight;
import org.firstinspires.ftc.teamcode.core.Pika;

@Autonomous
public class BlueSampleAuto extends SampleAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        Camlight.red = false;
        super.runOpMode();
    }

}
