package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class SpecDepositTeleOp extends SequentialCommand {
    public SpecDepositTeleOp() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_DEPOSIT.getTicks()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ArmMove(Arm.ArmPos.INTAKE.getPosition())
        );
    }
}
