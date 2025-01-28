package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class Hang extends SequentialCommand {
    public Hang() {
        super(
                new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.RETRACT.getPosition())),
                new Wait(300),
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.PREP_HANG.getPosition())
                ),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder()),
                new SlidesMove(OuttakeSlides.TurnValue.HANG.getTicks()),
                new MoveChassisTillStopped(),
                new ArmMove(Arm.ArmPos.STUPID_TAIL.getPosition()),
                new SlidesMove(OuttakeSlides.TurnValue.HANG_RETRACT.getTicks())
        );
    }
}
