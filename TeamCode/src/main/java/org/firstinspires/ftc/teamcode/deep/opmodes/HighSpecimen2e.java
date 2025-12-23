package org.firstinspires.ftc.teamcode.deep.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.deep.DeepBotAuto;

@Autonomous
public class HighSpecimen2e extends DeepBotAuto {

    DeepBot bot = new DeepBot();
    boolean stopEarly = true;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        autoWaitForStart();

//        bot.setPose(-4, 7.5, 90);

        // Set arm and deposit first specimen

        setArmForSpecimenHang4();

        driveTo(fast, -4,12, 90, 1);
        while(opModeIsActive() && armBusy()) continue;
        driveTo(fast, -4, 44, 90,0.5);


        deliverSpecimen2();
        bot.openClaw();
        sleep(300);

        // drive to pickup next specimen from wall

//        bot.setPose(bot.getPose().x, 41, 90);
        driveTo(fast, bot.getPose().x, 30, 90, 1);

        bot.setArmDegrees(45);
        while (opModeIsActive() && armBusy()) continue;

        driveTo(fast, 48, 30, 90, 1);
        turnTo(-90, 120, 12, 2);
//        double x = 64.8 - bot.sonicLeft.getDistanceSync(25, DistanceUnit.INCH);
//        if (Math.abs(x - bot.getPose().x) < 8){
//            bot.setPose(x, bot.getPose().y, Math.toDegrees(bot.getPose().h));
//            driveTo(normalSpeed, 48.5, 30, -90, 0.5);
//        }



        // drive into alliance wall and reset pose

        driveTo(normalSpeed, 48.5, 4, -90, 1);
        // changed target Y from 3
//        bot.setPose(bot.getPose().x, 7.5, -90);

        // backup, set arm position, pickup specimen from wall

        driveTo(normalSpeed, 48.5, 26, -90, 1);
        setArmForWallPickup2();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && (armBusy() || slideBusy() || et.milliseconds()< 500)) continue;

        driveTo(normalSpeed, 48.5, 19, -90, .5);
        bot.closeClaw();
        sleep(1000);
        setArmForSpecimenHang4();

        // hangs second specimen

        driveTo(fast, 2, 21,-90,1);
        turnTo(90, 120, 12, 2);
        driveTo(fast, 2, 43, 90, 0.5);
        deliverSpecimen2();
        bot.openClaw();
//        bot.setPose(bot.getPose().x, 41, Math.toDegrees(bot.getPose().h));
        sleep(300);

        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);

        // park!

        driveTo(fast, 12, 16, 90, 1);
        bot.setArmDegrees(DeepBot.MIN_ARM_DEGREES);
        driveTo(fast, 60, 16, 90, 1);
        driveTo(fast, 60, 10, 90, 1);




        while (opModeIsActive()){
            continue;
        }
    }
}
