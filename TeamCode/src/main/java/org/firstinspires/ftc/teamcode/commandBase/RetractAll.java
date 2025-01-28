package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class RetractAll extends SequentialCommand {
    public RetractAll() {
        super(
                new RunCommand(()->Pika.newClaw.setPivotOrientation(180)),
                new Wait(30),

                new ParallelCommand(
                        new RunCommand(()->Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.UP.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
                ),
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.AFTER_GRAB.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.BEFORE_GRAB.getPosition()))
                ),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder())


        );
    }
}
