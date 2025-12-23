package org.firstinspires.ftc.teamcode.deep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Predicate;
import org.firstinspires.ftc.teamcode.xdrive.XDrive;
import org.firstinspires.ftc.teamcode.xdrive.XDriveAuto;

public abstract class DeepBotAuto extends XDriveAuto {

    public DeepBot bot;

    public MotionProfile normalSpeed = new MotionProfile(10, 20, 10);

    public void setBot(DeepBot b){

        super.setBot(b);

        bot = b;
    }

    public void autoWaitForStart(){
        bot.setClawPosition(DeepBot.CLAW_CLOSED);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);
        bot.setWristPosition(0.25);
        bot.setYawStraight();
        telemetry.addData("Let Go Of Bot", "");
        telemetry.update();
        sleep(3000);
        Localizer loc = bot.getLocalizer();
        if (loc instanceof XDrive.OtosLocalizer) {
            ((XDrive.OtosLocalizer)loc).calibrateIMU();
            while (opModeInInit() && ((XDrive.OtosLocalizer)loc).getCalibrationProgress() > 0) {
                telemetry.addData("Calibrating...", "");
                telemetry.update();
            }
        }

        telemetry.addData("Press START when ready...", "");
        telemetry.update();
        waitForStart();
    }

    public boolean slideBusy(){
        return Math.abs((bot.slideMotor.getCurrentPosition() - bot.slideMotor.getTargetPosition()) ) > 10;
    }
    public boolean armBusy(){
        return Math.abs((bot.armMotor.getCurrentPosition() - bot.armMotor.getTargetPosition()) ) > 10;
    }

    public double getAvgLeftDistance(int num){
        double sum = 0;
        for (int i = 0; opModeIsActive() && i < num; i++){
            sum += bot.getLeftDistance();
        }
        return sum/num;
    }

    public double getAvgBackDistance(int num){
        double sum = 0;
        for (int i = 0; opModeIsActive() && i < num; i++){
            sum += bot.getBackDistance();
        }
        return sum/num;
    }

    public void deliverSampleHighBucket(){

        ElapsedTime et = new ElapsedTime();
        bot.setArmDegrees(70);

        BetaLog.d("DELIVERY SAMPLE HIGH BUCKET: ARM LOOP");

        while(opModeIsActive() && et.milliseconds()<5000) {
            int armTarget = bot.armMotor.getTargetPosition();
            int armCurrent = bot.armMotor.getCurrentPosition();
            BetaLog.dd("Arm", "target = %d   current  = %d ",
                    armTarget, armCurrent);
            if (Math.abs(armTarget -armCurrent) < 10){
                break;
            }
        }


        et.reset();
        bot.setSlideInches(44);


        BetaLog.d("DELIVERY SAMPLE HIGH BUCKET: SLIDE LOOP");

        while(opModeIsActive() && et.milliseconds()<5000) {
            int slideTarget = bot.slideMotor.getTargetPosition();
            int slideCurrent = bot.slideMotor.getCurrentPosition();
            BetaLog.dd("Slide", "target = %d   current  = %d ",
                    slideTarget, slideCurrent);
            if (Math.abs(slideTarget -slideCurrent) < 10){
                break;
            }
        }
        bot.setWristPosition(0.8);
        sleep(600);
        bot.setClawPosition(DeepBot.CLAW_WIDE_OPEN);
        sleep(500);
    }

    public void waitForMotor(Predicate condition){
        while(opModeIsActive()){
            if (condition.test()){
                break;
            }
        }
    }

    public void dropSampleInBucket(){
//        bot.setWristPosition(0.9);
//        sleep(800);
        bot.setClawPosition(DeepBot.CLAW_WIDE_OPEN);
        sleep(500);
        bot.setWristPosition(0.25);
        sleep(300);
    }


    public void dropSampleInBucket2(){
//        bot.setWristPosition(0.9);
//        sleep(800);
        bot.setClawPosition(DeepBot.CLAW_WIDE_OPEN);
        sleep(300);
        bot.setWristPosition(0.25);
        sleep(300);
    }



    public void driveLeftDist(MotionProfile mProfile, double targetDist, double targetY,
                        double targetHeadingDegrees, double tolerance){
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        Pose startPose = bot.getPose();

        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            double d2 = Math.abs(targetY - pose.y);

            if (d2 < tolerance){
                break;
            }

            double vY = mProfile.vMin;
            double vX = 4 * (targetDist - bot.getLeftDistance());
            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vXR = vX * sin - vY * cos;
            double vYR = vX * cos + vY * sin;

            double vA = 2.0 * AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            bot.setDriveSpeed(vXR, vYR, vA);
        }

        bot.setDrivePower(0, 0, 0);
    }

    public void setArmForWallPickup(){
        bot.setSlideInches(20);
        bot.setArmDegrees(-17
        ); // Was -16.5
        bot.setWristPosition(0.5);
        bot.openClaw();
    }

    public void setArmForWallPickup2(){
        bot.setSlideInches(20);
        bot.setArmDegrees(-15);
        bot.setWristPosition(0.45);
        bot.openClaw();
    }

    public void setArmForSpecimenHang1(){
        bot.setArmDegrees(21);
        bot.setSlideInches(20);
        bot.setWristPosition(0.5);
    }

    public void setArmForSpecimenHang2(){
        bot.setArmDegrees(24);  // was 23.5
        bot.setSlideInches(21);
        bot.setWristPosition(0.5);
    }

    public void
    deliverSpecimen(){
        bot.setArmDegrees(23);
        while (opModeIsActive() && armBusy() ) continue;
        bot.setSlideInches(16);
    }

    public void deliverSpecimen2(){
        bot.setArmDegrees(10);
        while (opModeIsActive() && armBusy()) continue;
    }


    public void setArmForSpecimenHang3(){
        bot.setArmDegrees(55);
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);
        bot.setWristPosition(0.7);
    }

    public void  setArmForSpecimenHang4(){
        bot.setArmDegrees(45.6);  //Was 42.6 then 43.6
        bot.setSlideInches(DeepBot.SLIDE_BASE_LENGTH);
        bot.setWristPosition(0.6);
    }


//    public double getXFromSonic(){
//        return 64.0 - bot.sonicLeft.getDistanceAsync(25, DistanceUnit.INCH);
//    }
//
//    public class SonicXAdjuster implements Runnable{
//        public SonicXAdjuster(){
//            bot.sonicLeft.getDistanceAsync(0, DistanceUnit.INCH);
//        }
//        public void run(){
//            double dist = bot.sonicLeft.getDistanceAsync(25, DistanceUnit.INCH);
//            if (dist<0 || bot.getPose().y < 16){
//                return;
//            }
//            double xMeas = 64.8 - dist;
//            if (Math.abs(xMeas - bot.getPose().x) > 4){
//                return;
//            }
//            double xEst = 0.25*xMeas + 0.75*bot.getPose().x;
//            bot.setPose(xEst, bot.getPose().y, Math.toDegrees(bot.getPose().h));
//        }
//    }


}
