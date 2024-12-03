package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.Command;
import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class PrepareOuttake extends SequentialCommand {
    public PrepareOuttake() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ParallelCommand(
                        new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.PLACE.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition()))
                ),

                new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),
                new SlidesMove(OuttakeSlides.TurnValue.PREPARE_OUTTAKE.getTicks())
        );
    }
}
