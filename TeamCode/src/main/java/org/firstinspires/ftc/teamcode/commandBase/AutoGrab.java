package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.core.Pika;

public class AutoGrab extends SequentialCommand{
    public AutoGrab() {
        super(
                new Wait(200),
                new RunCommand(() -> Pika.newClaw.setPivotOrientation(Pika.limelight.getSampleOrientation())),
                new Wait(250),
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.GRAB.getPosition())),
                new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.GRAB.getPosition())),
                new Wait(150),
                new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new Wait(150),
                new ParallelCommand(
                        new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setPivotOrientation(90))
                )
        );
    }
}
