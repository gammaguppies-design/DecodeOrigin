package org.firstinspires.ftc.teamcode.deep.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.deep.DeepBotAuto;

@Autonomous
@Disabled
public class HighSpecimen3 extends DeepBotAuto {

    DeepBot bot = new DeepBot();
    boolean stopEarly = true;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        autoWaitForStart();

//        bot.setPose(0, 7.5, 90);

        // Set arm and deposit first specimen

        setArmForSpecimenHang3();

        driveTo(fast, 0,12, 90, 1);
        while(opModeIsActive() && armBusy()) continue;
        driveTo(fast, 0, 44, 90,0.5);


        deliverSpecimen();
        bot.openClaw();
        sleep(300);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);


        // Drive to sample on spike mark



//        bot.setPose(bot.getPose().x, 41, 90);

        splineTo(fast, 36, 41, 90, 1, true);
        driveLine(fast, new VectorF(36,41),
                new VectorF(36, 60), -90,1);
        driveLine(fast, new VectorF(36,60),
                new VectorF(47, 60), -90, 1);
        while (opModeIsActive()) {
            telemetry.addData("pose", "x %.1f  y %.1f  h %.1f", bot.getPose().x,
                    bot.getPose().y, Math.toDegrees(bot.getPose().h));
            telemetry.update();
        }

//        double x = 64.8 - bot.sonicLeft.getDistanceSync(25, DistanceUnit.INCH);
//        if (Math.abs(x - bot.getPose().x) < 8){
//            bot.setPose(x, bot.getPose().y, Math.toDegrees(bot.getPose().h));
//            driveTo(normalSpeed, 48.5, 30, -90, 0.5);
//        }

//        TODO: use left distance sensor to adjust x and adjust bot position


        // drive into alliance wall and reset pose

        bot.setArmDegrees(30);
        driveTo(normalSpeed, 48.5, 3, -90, 1);
//        bot.setPose(bot.getPose().x, 7.5, -90);

        // backup, set arm position, pickup specimen from wall

        driveTo(normalSpeed, 48.5, 26, -90, 1);
        setArmForWallPickup();
        sleep(500);
        driveTo(normalSpeed, 48.5, 19, -90, .5);
        bot.closeClaw();
        sleep(700);
        setArmForSpecimenHang3();

        // hangs second specimen

        driveTo(fast, 6, 21,-90,1);
        turnTo(90, 120, 12, 2);
        driveTo(fast, 6, 43, 90, 0.5);
        deliverSpecimen();
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
