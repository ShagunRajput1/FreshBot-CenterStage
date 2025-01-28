package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;
import org.firstinspires.ftc.teamcode.pathing.MotionPlannerEdit;

public class AutoIntakeFromSub extends SequentialCommand {
    public AutoIntakeFromSub(MotionPlannerEdit follower) {
        super(
                new SlidesMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                new RunCommand(()-> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFORE_GRAB.getPosition())),
                new RunCommand(()-> Pika.newClaw.setMiniPitch(FinalClaw.MiniPitch.DETECT.getPosition())),

                new ParallelCommand(
                        new ArmMove(Arm.ArmPos.INTAKE.getPosition()),
                        new RunCommand(()-> Pika.newClaw.setPivotOrientation(90)),
                        new RunCommand(()-> Pika.newClaw.setClaw(FinalClaw.ClawPosition.OPEN.getPosition()))
                ),
                new RunCommand(() ->Pika.outtakeSlides.resetEncoder()),
                new SlidesMove(4750),
                new AlignWithSample(follower),
                new Wait(500),
                new SpecialGrab(),
                new RunCommand(()->Pika.outtakeSlides.resume()),
                new RetractAll(),
                new RunCommand(follower::resume)
        );
    }

}
