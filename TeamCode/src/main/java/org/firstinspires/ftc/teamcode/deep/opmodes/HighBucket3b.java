package org.firstinspires.ftc.teamcode.deep.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.deep.DeepBotAuto;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SavedData;

@Autonomous

public class HighBucket3b extends DeepBotAuto {

    DeepBot bot = new DeepBot();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);

        autoWaitForStart(); // sets stuff up for auto

//        bot.setPose(-31, 8, 180); // sets pose

        bot.setArmDegrees(67); // sets arm

        // drives to basket

        driveTo(fast, -31, 17.7, 180, 1);
        if (bot.getArmAngle()> 45){ // extends slide if arm is !busy
            bot.setSlideInches(44);
        }
        driveTo(fast, -52.3, 17.7, 180, 1);
        if (bot.getArmAngle()> 45){ // extends slide if arm is !busy
            bot.setSlideInches(44);
        }
        bot.setWristPosition(0.764); // sets wrist position
        turnTo(-135, 120, 12, 2);

        // wait for arm and slide to be in position

        bot.armMotor.getCurrentPosition(); //
        while (opModeIsActive() && armBusy()) continue;
        bot.slideMotor.getCurrentPosition();
        bot.setSlideInches(44);
        while (opModeIsActive() && slideBusy()) continue;

        // drop first sample into bucket

        dropSampleInBucket2();

        // retract slide and wait until less than 24 inches

        bot.slideMotor.getCurrentPosition();
        bot.setSlideInches(16);
        waitForMotor(()-> bot.getSlideInches() < 24);


        turnTo(90, 120, 12,2);

        // set arm and wrist to pickup sample from floor

        bot.setArmDegrees(-33.5);
        bot.setWristPosition(0.7); // was 0.764
        bot.slideMotor.getCurrentPosition();
        bot.armMotor.getCurrentPosition();
        while (opModeIsActive()){
            if (!armBusy() && !slideBusy()) break;
        }

        // get distance to wall and determine target x value

        double lDist = getAvgLeftDistance(5);
        double x = bot.getPose().x;
        double targetX = x + 16.5 - lDist; // was 16.5

        // drive to pickup second sample

        driveTo(fast, targetX, bot.getPose().y, 90, 0.5);
        Pose pose1 = new Pose(targetX, bot.getPose().y, bot.getPose().h);
        Pose pose2 = new Pose(targetX, 33, Math.toRadians(90));
        driveLine(normalSpeed, pose1, pose2, 0.5);

        // grab second sample from floor

        bot.setArmDegrees(-36.5);
        bot.armMotor.getCurrentPosition();
        while (opModeIsActive()){
            if (!armBusy() ) break;
        }
        bot.setWristPosition(0.764);
        sleep(200);
        bot.setClawPosition(DeepBot.CLAW_CLOSED);
        sleep(300);
//        bot.setPose(-49, 32, Math.toDegrees(bot.getPose().h));

        // raising the arm to deliver second sample

        bot.setArmDegrees(67);

        // drive to bucket with second sample

        driveTo(fast, -53.5, 19.5, 90, 1);
        if (bot.getArmAngle()> 45){
            bot.setSlideInches(44);
        }
        turnTo(-135,120,12, 2);

        // wait for arm elevation then extend slide and wait for slide

        bot.armMotor.getCurrentPosition();
        while (opModeIsActive() && armBusy() ) continue;
        bot.setSlideInches(44);
        bot.slideMotor.getCurrentPosition();
        while (opModeIsActive() && slideBusy()) continue;

        // drop second sample in bucket

        dropSampleInBucket2();

        // retract slide and wait until less than 24 inches

        bot.slideMotor.getCurrentPosition();
        bot.setSlideInches(16);
        waitForMotor(()-> bot.getSlideInches() < 24);


        turnTo(90, 120, 12,2);

        // set arm and wrist to pickup third sample form floor, then wait for arm and slide

        bot.setArmDegrees(-33.5);
        bot.setWristPosition(0.7); // was 0.764
        bot.slideMotor.getCurrentPosition();
        bot.armMotor.getCurrentPosition();
        while (opModeIsActive()){
            if (!armBusy() && !slideBusy()) break;
        }

        // measure distance and get target x for picking up third sample

        lDist = getAvgLeftDistance(5);
        x = bot.getPose().x;
        targetX = x + 6 - lDist; // was 6.5

        // drive to pickup third sample

        driveTo(normalSpeed, targetX, bot.getPose().y, 90, 0.5);
        pose1 = new Pose(targetX, bot.getPose().y, bot.getPose().h);
        pose2 = new Pose(targetX, 32, Math.toRadians(90));
        driveLine(normalSpeed, pose1, pose2, 0.5);

        // pick up third sample
        bot.setArmDegrees(-36.5);
        bot.armMotor.getCurrentPosition();
        while (opModeIsActive()){
            if (!armBusy()) break;
        }
        bot.setWristPosition(0.764);
        sleep(200);
        bot.setClawPosition(DeepBot.CLAW_CLOSED);
        sleep(300);
//        bot.setPose(-59, 32, Math.toDegrees(bot.getPose().h));

        // sets arm to drop off third sample

        bot.setArmDegrees(67);

        // driving to bucket with third sample

        driveTo(fast, -54.5, 20.5, 90, 1);
        if (bot.getArmAngle()>45){
            bot.setSlideInches(44);
        }
        turnTo(-135,120,12, 2);

        // wait for arm then extends slide and wait for slide

        bot.armMotor.getCurrentPosition();
        while (opModeIsActive() && armBusy() ) continue;
        bot.setSlideInches(44);
        bot.slideMotor.getCurrentPosition();
        while (opModeIsActive() && slideBusy()) continue;

        // drops third sample off

        dropSampleInBucket2();

        // retract slide

        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);

        // save pose for begin of teleop

        SavedData.pose = bot.getPose();

        // wait until opmode times out to allow arm to retract




        while (opModeIsActive()) continue;

    }
}
