package org.firstinspires.ftc.teamcode.commandBase;

import com.google.gson.FieldNamingPolicy;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class PreparePark extends SequentialCommand {
    public PreparePark() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),

                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(180)),
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.APRIL.getPosition())),
                        new RunCommand(()->Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition()))
                ),
                new RunCommand(()->Pika.outtakeSlides.resetEncoder())

        );
    }
}
