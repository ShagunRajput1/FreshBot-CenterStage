package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class Grab extends SequentialCommand {
    public Grab() {
        super(
                new Wait(1000),
                new RunCommand(() -> Pika.claw.setArmPitch(Claw.ArmPitch.GRAB.getPosition())),
                new Wait(1000),
                new RunCommand(() -> Pika.claw.setClaw(Claw.ClawPosition.CLOSE.getPosition())),
                new RunCommand(() -> Pika.claw.setArmPitch(Claw.ArmPitch.UP.getPosition()))

                );
    }

}
