package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class IntakeSpecWall extends SequentialCommand {
    public IntakeSpecWall() {
        super(
                new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                new ParallelCommand(
                        new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.SPEC_WALL.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition()))
                ),

                new RunCommand(()-> Pika.outtakeSlides.setMaxPower(1)),
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),

//            new RunCommand(() -> Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),

                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.SPEC_WALL.getPosition()))
                ),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder())
        );
    }
}
