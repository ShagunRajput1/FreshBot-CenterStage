package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class DepositSample extends SequentialCommand {
    public DepositSample(int slidePos) {
        super (
            new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
            new SlidesMove(slidePos),
            new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.DEPOSIT.getPosition())),
            new Wait(100),
            new RunCommand(()->Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition()))
        );
    }
}
