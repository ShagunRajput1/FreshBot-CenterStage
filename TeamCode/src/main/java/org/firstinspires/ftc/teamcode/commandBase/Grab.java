package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.commandSystem.Wait;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.FinalClaw;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class Grab extends SequentialCommand {
    public Grab() {
        super(
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.GRAB.getPosition())),
                new Wait(500),
                new RunCommand(() -> Pika.newClaw.setClaw(FinalClaw.ClawPosition.CLOSE.getPosition())),
                new RunCommand(() -> Pika.newClaw.setArmPitch(FinalClaw.ArmPitch.BEFOREGRAB.getPosition()))

                );
    }

}
