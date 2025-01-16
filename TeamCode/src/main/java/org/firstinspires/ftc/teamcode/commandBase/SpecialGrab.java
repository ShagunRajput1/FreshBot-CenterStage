package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

public class SpecialGrab extends SequentialCommand {

    public SpecialGrab() {
        super(
                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.GRAB.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.GRAB.getPosition()))
                ),
                new Wait(300),
                new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new Wait(100),
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition()))
        );
    }
}
