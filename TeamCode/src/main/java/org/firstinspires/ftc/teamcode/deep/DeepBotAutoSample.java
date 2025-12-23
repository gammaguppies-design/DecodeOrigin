package org.firstinspires.ftc.teamcode.deep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DeepBotAutoSample extends DeepBotAuto{

    DeepBot bot = new DeepBot();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        waitForStart();

    }
}
