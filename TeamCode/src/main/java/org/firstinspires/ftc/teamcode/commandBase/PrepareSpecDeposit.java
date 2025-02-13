package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class PrepareSpecDeposit extends SequentialCommand {
    public PrepareSpecDeposit() {
        super(

                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_DEPOSIT.getPosition())),
                new ParallelCommand(
                        new SequentialCommand(
                                new ArmMove(Arm.ArmPos.SPEC_DEPOSIT.getPosition()),
                                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.LITTLE_OPEN.getPosition()))
                        ),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_DEPOSIT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                        new RunCommand(()->Pika.outtakeSlides.setMaxPower(0.85))
                ),
                new RunCommand(()-> Pika.outtakeSlides.resetEncoder()),
                new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new SlidesMove(OuttakeSlides.TurnValue.SPEC_PREP.getTicks())
        );
    }
}
