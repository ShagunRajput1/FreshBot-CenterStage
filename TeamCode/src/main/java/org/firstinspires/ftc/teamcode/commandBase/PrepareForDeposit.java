package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class PrepareForDeposit extends SequentialCommand {
    public PrepareForDeposit() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ParallelCommand(
                        new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setPivotOrientation(180))
                ),

                new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder()),
                new SlidesMove(OuttakeSlides.TurnValue.BUCKET2.getTicks())
        );
    }
}
