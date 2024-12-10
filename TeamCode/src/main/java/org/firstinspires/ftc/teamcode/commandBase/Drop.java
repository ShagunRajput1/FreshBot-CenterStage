package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

public class Drop extends SequentialCommand {
    public Drop() {
        super(
                new ParallelCommand(
                    new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.DEPOSIT.getPosition())),
                    new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DEPOSIT.getPosition()))
                ),
                new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition()))
        );
    }

}
