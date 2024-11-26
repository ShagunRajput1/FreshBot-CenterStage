package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class RetractAll extends SequentialCommand {
    public RetractAll() {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.INTAKE.getPosition())
                ),
                new RunCommand(()-> Pika.claw.setArmPitch(Claw.ArmPitch.UP.getPosition()))


        );
    }
}
