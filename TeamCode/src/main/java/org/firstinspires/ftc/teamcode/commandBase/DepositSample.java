package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class DepositSample extends SequentialCommand {
    public DepositSample(int slidePos) {
        super (
//            new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
//            new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
//            new RunCommand(()->Pika.outtakeSlides.resetEncoder()),
//            new SlidesMove(slidePos),
            new ParallelCommand(
                    new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.DEPOSIT.getPosition())),
                    new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DEPOSIT.getPosition())),
                    new RunCommand(()-> Pika.newClaw.setPivotOrientation(90))

            ),
            new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
            new Wait(250),
            new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
            new Wait(50)
        );
    }
}
