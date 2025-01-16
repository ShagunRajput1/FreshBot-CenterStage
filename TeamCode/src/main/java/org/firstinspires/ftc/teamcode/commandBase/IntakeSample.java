package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class IntakeSample extends SequentialCommand {
    public IntakeSample(int slidePos) {
        super (
                new SlidesMove(slidePos),
                new ParallelCommand(
                    new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                    new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.RETRACT.getPosition())),
                    new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.GRAB.getPosition())),
                    new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                    new ArmMove(Arm.ArmPos.INTAKE.getPosition())
            )

        );
    }
}
