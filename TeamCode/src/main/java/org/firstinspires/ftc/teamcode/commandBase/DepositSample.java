package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class BucketSample extends SequentialCommand {
    public BucketSample(int slidePos) {
        super(
            new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
            new SlidesMove(OuttakeSlides.TurnValue.TEST.getTicks())
        );
    }
}
