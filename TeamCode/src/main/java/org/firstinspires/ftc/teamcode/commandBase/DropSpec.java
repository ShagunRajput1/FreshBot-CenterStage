package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class DropSpec extends SequentialCommand {
    public DropSpec() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.SPEC_DEPOSIT.getPosition()),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.DEPOSIT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DEPOSIT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(180))
                ),
                new RunCommand(()-> Pika.outtakeSlides.resetEncoder())
        );
    }
}
