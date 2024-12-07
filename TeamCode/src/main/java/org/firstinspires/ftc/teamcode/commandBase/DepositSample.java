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
            new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
            new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
            new SlidesMove(slidePos),
            new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
            new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.PLACE.getPosition()))
        );
    }
}
