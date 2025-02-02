package org.firstinspires.ftc.teamcode.commandBase;

import org.firstinspires.ftc.teamcode.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Pika;

public class AutoIntakeFromSubBasic extends SequentialCommand {
    public AutoIntakeFromSubBasic() {
        super(
                new RunCommand(()-> Pika.newClaw.setPivotOrientation(90)),
                new AlignWithSampleBasic(),
                new TeleGrab(),
                new RunCommand(()-> Pika.outtakeSlides.resume())
        );
    }
}
