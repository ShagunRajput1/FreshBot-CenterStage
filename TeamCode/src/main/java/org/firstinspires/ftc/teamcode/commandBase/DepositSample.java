package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class DepositSample extends SequentialCommand {
    public DepositSample(int slidePos) {
        super (
            new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
//            new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.UP.getPosition())),
            new ParallelCommand (
                new RunCommand(()->Pika.claw.setPitch(Claw.PitchPosition.PLACE.getPosition()[0],
                        Claw.PitchPosition.PLACE.getPosition()[1])),
                new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.UP.getPosition()))
            ),



            new ArmMove(Arm.ArmPos.OUTTAKE.getPosition()),

            new SlidesMove(slidePos),
            new Wait(1000),
            new RunCommand(()->Pika.claw.setArmPitch(Claw.ArmPitch.DEPOSIT.getPosition()))
        );
    }
}
