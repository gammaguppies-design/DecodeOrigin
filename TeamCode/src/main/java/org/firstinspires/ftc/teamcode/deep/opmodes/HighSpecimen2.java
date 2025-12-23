package org.firstinspires.ftc.teamcode.deep.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.deep.DeepBotAuto;

@Autonomous
@Disabled
public class HighSpecimen2 extends DeepBotAuto {

    DeepBot bot = new DeepBot();
    boolean stopEarly = true;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        autoWaitForStart();
//
//        bot.setPose(0, 7.5, 90);

        // Set arm and deposit first specimen

        setArmForSpecimenHang1();

        driveTo(normalSpeed, 0, 39, 90, 1);
        driveTo(normalSpeed, 0, 42, 90, 0.5);
        bot.openClaw();
        sleep(300);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);


        // Drive to sample on spike mark



//        bot.setPose(bot.getPose().x, 41, 90);
        driveTo(fast, bot.getPose().x, 30, 90, 1);
        driveTo(fast, 34, 30, 90, 1);
        driveTo(fast, 34, 60, 90, 1);
        driveTo(fast, 46.5, 60, 90, 1);
        turnTo(-90, 90, 8, 2);

//        TODO: use left distance sensor to adjust x and adjust bot position


        // push sample into alliance wall and reset pose

        bot.setArmDegrees(30);
        driveTo(fast, 46.5, 9, -90, 1);
        driveTo(slow, 46.5 , 1, -90, 1);
//        bot.setPose(bot.getPose().x, 7.5, -90);

        // backup, set arm position, pickup specimen from wall

        driveTo(normalSpeed, 46.5, 26, -90, 1);
        setArmForWallPickup();
        driveTo(normalSpeed, 46.5, 18, -90, .5);
        bot.closeClaw();
        sleep(400);
        setArmForSpecimenHang2();

        // hangs second specimen

        driveTo(fast, 6, 21,-90,1);
        turnTo(90, 90, 8, 2);
        driveTo(normalSpeed, 6, 43, 90, 0.5);
        bot.openClaw();
        sleep(300);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);

        // park!

        driveTo(fast, 12, 16, 90, 1);
        bot.setArmDegrees(DeepBot.MIN_ARM_DEGREES);
        driveTo(fast, 48, 16, 90, 1);




        while (opModeIsActive()){
            continue;
        }
    }
}
