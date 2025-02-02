package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class SpecPark extends SequentialCommand {
    public SpecPark() {
        super (
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),

//            new RunCommand(() -> Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),
                new ParallelCommand(
                        new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.RETRACT.getPosition()))
                ),

                new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder()),
                new SlidesMove(6000)
        );
    }
}
