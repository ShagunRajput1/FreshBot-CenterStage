package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.digitalchickenlabs.CachingOctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Camlight;
import org.firstinspires.ftc.teamcode.core.Pika;

@Autonomous
public class RedSampleAuto extends SampleAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        Camlight.red = true;
        super.runOpMode();
    }

}
