package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

public class Grab extends SequentialCommand {
    public Grab() {
        super(
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.GRAB.getPosition())),
                new Wait(200),
                new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new Wait(200),
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition()))

                );
    }

}
