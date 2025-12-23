package org.firstinspires.ftc.teamcode.deep.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.deep.DeepBotAuto;

@Autonomous
@Disabled
public class HighSpecimen2b extends DeepBotAuto {

    DeepBot bot = new DeepBot();
    boolean stopEarly = true;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        autoWaitForStart();

//        bot.setPose(0, 7.5, 90);

        // Set arm and deposit first specimen

        setArmForSpecimenHang1();

        driveTo(fast, 0, 39, 90, 1);
        driveTo(normalSpeed, 0, 42, 90, 0.5);
        bot.openClaw();
        sleep(300);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);


        // Drive to sample on spike mark



//        bot.setPose(bot.getPose().x, 41, 90);
        driveTo(normalSpeed, bot.getPose().x, 30, 90, 1);
        driveTo(fast, 32, 30, 90, 1);
        driveTo(fast, 32, 60, 90, 1);
        driveTo(normalSpeed, 47.5, 60, 90, 1);
        turnTo(-90, 90, 8, 2);
//        double x = 64.8 - bot.sonicLeft.getDistanceSync(25, DistanceUnit.INCH);
//        if (Math.abs(x - bot.getPose().x) < 8){
//            bot.setPose(x, bot.getPose().y, Math.toDegrees(bot.getPose().h));
//            driveTo(slow, 48.5, 60, -90, 0.5);
//        }

//        TODO: use left distance sensor to adjust x and adjust bot position


        // push sample into alliance wall and reset pose

        bot.setArmDegrees(30);
        driveTo(fast, 48.5, 9, -90, 1);
        driveTo(slow, 48.5 , 1, -90, 1);
//        bot.setPose(bot.getPose().x, 7.5, -90);

        // backup, set arm position, pickup specimen from wall

        driveTo(normalSpeed, 48.5, 26, -90, 1);
        setArmForWallPickup();
        sleep(500);
        driveTo(normalSpeed, 48.5, 19, -90, .5);
        bot.closeClaw();
        sleep(700);
        setArmForSpecimenHang2();

        // hangs second specimen

        driveTo(normalSpeed, 6, 21,-90,1);
        turnTo(90, 90, 8, 2);
        driveTo(fast, 6, 43, 90, 0.5);
        bot.openClaw();
//        bot.setPose(bot.getPose().x, 41, Math.toDegrees(bot.getPose().h));
        sleep(300);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);

        // park!

        driveTo(fast, 12, 16, 90, 1);
        bot.setArmDegrees(DeepBot.MIN_ARM_DEGREES);
        driveTo(fast, 60, 16, 90, 1);




        while (opModeIsActive()){
            continue;
        }
    }
}
