package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class IntakeSample extends SequentialCommand {
    public IntakeSample() {
        super (
            new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
//            new RunCommand(() -> Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),
            new ParallelCommand(
                    new RunCommand(() -> Pika.claw.setClaw(Claw.ClawPosition.OPEN.getPosition())),
                    new RunCommand(() -> Pika.claw.setPitch(Claw.PitchPosition.GRAB.getPosition()[0], Claw.PitchPosition.GRAB.getPosition()[1]))
            ),
            new Wait(500),

            new RunCommand(()-> Pika.claw.setArmPitch(Claw.ArmPitch.DOWN.getPosition())),
            new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
            new Wait(1000)
        );
    }
}
