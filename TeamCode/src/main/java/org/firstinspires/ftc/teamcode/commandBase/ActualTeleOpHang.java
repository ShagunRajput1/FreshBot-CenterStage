package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class ActualTeleOpHang extends SequentialCommand {
    public ActualTeleOpHang() {
        super(
                new ParallelCommand(
                        new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.RETRACT.getPosition())),
                        new RunCommand(() -> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.RETRACT.getPosition())),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(180))
                ),
                new Wait(300),
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new RunCommand(()-> Pika.outtakeSlides.resetEncoder()),
                new ArmMove(Arm.ArmPos.PREP_HANG.getPosition()),
                new SlidesMove(OuttakeSlides.TurnValue.HANG.getTicks()),
                new ArmMove(Arm.ArmPos.STUPID_TAIL.getPosition()),

                new ParallelCommand(
                        new SlidesMove(OuttakeSlides.TurnValue.HANG_RETRACT.getTicks()),
                        new SequentialCommand(
                                new Wait(500),
                                new ArmMove(Arm.ArmPos.HANG.getPosition())
                        )
                )
        );
    }
}
