package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.List;

public class XDrive{

    // TODO: Set values of TICKS_PER_INCH, STRAFE_RATIO, TICKS_PER_RAD, and MAX_TICKS_PER_SEC for the actual robot
    public static final double TICKS_PER_INCH = 33.69; // Ticks per inch (TPI) of forward travel
    public static final double STRAFE_RATIO = 1.0; // Ratio of TPI for strafe to TPI for forward travel
    public static final double TICKS_PER_RAD = 340.31; // Ticks per radian that robot heading changes
    public static final double MAX_TICKS_PER_SEC = 2500; // Maximum achievable Ticks per Second
    public static final double SQRT2 = Math.sqrt(2);

    public DcMotorEx bl, fl, br, fr;
    public IMU imu;
    double headingOffsetRadians = 0;




    public Localizer localizer;
    public OtosLocalizer otosLoc;
    private void setHeadingRadians(double headingRadians){
        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        headingOffsetRadians = AngleUnit.normalizeRadians(headingRadians - rawHeading);
    }

//    public void setspose(double x, double y, double hDegrees) {
//        double hRadians = Math.toRadians(hDegrees);
//        setHeadingRadians(hRadians);
//        ticksBL = bl.getCurrentPosition();
//        ticksFL = fl.getCurrentPosition();
//        ticksFR = fr.getCurrentPosition();
//        ticksBR = br.getCurrentPosition();
//        pose = new Pose(x, y, hRadians);
//    }
    public void init(HardwareMap hwMap){

        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub: allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Get the DC Motors
        bl = hwMap.get(DcMotorEx.class, "leftBackDrive");
        fl = hwMap.get(DcMotorEx.class, "leftFrontDrive");
        fr = hwMap.get(DcMotorEx.class, "rightFrontDrive");
        br = hwMap.get(DcMotorEx.class, "rightBackDrive");

        // Set zero power behavior to BRAKE for all motors (so robot stops quickly)
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Set Direction to Reverse for the correct motors
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset all drive motor encoders (this also disables them)
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Re-enable motors and set them to RUN_USING_ENCODER mode
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

         //TODO: Set RevHub orientation correctly for the real robot
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        ElapsedTime et = new ElapsedTime();
        while (et.milliseconds() < 150 && !Thread.currentThread().isInterrupted()) continue;


        localizer = new DriveLocalizer();


//        SparkFunOTOS otos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
//        otosLoc = new OtosLocalizer(otos);
    }

    public Localizer getLocalizer(){
        return localizer;
    }


    public Pose getPose(){ return localizer.getPose(); }

//    public void setpose(double x, double y, double hDegrees){
//        localizer.setPose(x, y, hDegrees);
//    }

    public Pose getVelocity(){
        return localizer.getVelocity();
    }

    public void refreshPose(){
        if (localizer instanceof OtosLocalizer) {
            ((OtosLocalizer)localizer).refreshPose();
        }
    }

    public void setDrivePower(double px, double py, double pa){
        double pBL = py - px - pa;
        double pFL = py + px - pa;
        double pFR = py - px + pa;
        double pBR = py + px + pa;

        double max = Math.max(1, Math.abs(pBL));
        max = Math.max(max, Math.abs(pFL));
        max = Math.max(max, Math.abs(pFR));
        max = Math.max(max, Math.abs(pBR));

        if (max > 1){
            pBL /= max;
            pFL /= max;
            pFR /= max;
            pBR /= max;
        }

        bl.setPower(pBL);
        fl.setPower(pFL);
        fr.setPower(pFR);
        br.setPower(pBR);
    }

    public void setDriveSpeed(double vx, double vy, double va){
        double px = vx * TICKS_PER_INCH * STRAFE_RATIO / MAX_TICKS_PER_SEC;
        double py = vy * TICKS_PER_INCH / MAX_TICKS_PER_SEC;
        double pa = va * TICKS_PER_RAD / MAX_TICKS_PER_SEC;
        setDrivePower(px, py, pa);
    }

    public void updateOdometry(){
        localizer.update();
    }




    public class OtosLocalizer implements Localizer {
        public SparkFunOTOS otos;
        public Pose pose = new Pose(0,0,0);
        public Pose velocity = new Pose(0,0,0);

        public OtosLocalizer(SparkFunOTOS otos){
            this.otos = otos;
            otos.setAngularUnit(AngleUnit.RADIANS);
            otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(50)));
            otos.setLinearScalar(1.04);
            otos.resetTracking();
        }

        public void calibrateIMU(){
            otos.calibrateImu();
        }

        public int getCalibrationProgress(){
            return otos.getImuCalibrationProgress();
        }

        public void refreshPose(){
            otos.resetTracking();
            setPose(pose.x, pose.y, Math.toDegrees(pose.h));
        }

        public void setPose(double x, double y, double headingDegrees){

        }

        public Pose getPose(){
            return pose;
        }

        public Pose getVelocity(){
            return velocity;
        }

        public void update(){
            SparkFunOTOS.Pose2D pose2D = otos.getPosition();
            pose = new Pose(pose2D.x, pose2D.y, pose2D.h);

            double vBL = bl.getVelocity();
            double vFL = fl.getVelocity();
            double vFR = fr.getVelocity();
            double vBR = br.getVelocity();

            double vXR = 0.25 * (-vBL + vFL - vFR + vBR) / (TICKS_PER_INCH * STRAFE_RATIO);
            double vYR = 0.25 * (vBL + vFL + vFR + vBR) / TICKS_PER_INCH;
            double vA = 0.25 * (-vBL - vFL + vFR + vBR) / TICKS_PER_RAD;

            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vX = vXR * sin + vYR * cos;
            double vY = -vXR * cos + vYR * sin;

            velocity = new Pose(vX, vY, vA);
        }
    }

    public class DriveLocalizer implements Localizer{
        public Pose pose = new Pose(0,0,0);
        public Pose velocity = new Pose(0,0,0);

        public double headingOffsetRadians = 0;
        private int ticksBL = 0, ticksFL = 0, ticksFR = 0, ticksBR = 0;


        private void setHeadingRadians(double headingRadians){
            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingOffsetRadians = AngleUnit.normalizeRadians(headingRadians - rawHeading);
        }

        private double getHeadingRadians(){
            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            return AngleUnit.normalizeRadians(headingOffsetRadians + rawHeading);
        }

        public Pose getPose(){ return pose; }

        public void setPose(double x, double y, double hDegrees){
            double hRadians = Math.toRadians(hDegrees);
            setHeadingRadians(hRadians);
            ticksBL = bl.getCurrentPosition();
            ticksFL = fl.getCurrentPosition();
            ticksFR = fr.getCurrentPosition();
            ticksBR = br.getCurrentPosition();
            pose = new Pose(x, y, hRadians);
        }


        public Pose getVelocity(){
            return velocity;
        }


        public void update(){
            int currTicksBL = bl.getCurrentPosition();
            int currTicksFL = fl.getCurrentPosition();
            int currTicksFR = fr.getCurrentPosition();
            int currTicksBR = br.getCurrentPosition();

            int newBL = currTicksBL - ticksBL;
            int newFL = currTicksFL - ticksFL;
            int newFR = currTicksFR - ticksFR;
            int newBR = currTicksBR - ticksBR;

            ticksBL = currTicksBL;
            ticksFL = currTicksFL;
            ticksFR = currTicksFR;
            ticksBR = currTicksBR;

            double dXR = 0.25 * (-newBL + newFL - newFR + newBR) / (TICKS_PER_INCH * STRAFE_RATIO);
            double dYR = 0.25 * (newBL + newFL + newFR + newBR) / TICKS_PER_INCH;

            double heading = getHeadingRadians();
            double deltaHeading = AngleUnit.normalizeRadians(heading - pose.h);
            double avgHeading = AngleUnit.normalizeRadians(pose.h + deltaHeading/2.0);
            double sinAvg = Math.sin(avgHeading);
            double cosAvg = Math.cos(avgHeading);

            double dX = dXR * sinAvg + dYR * cosAvg;
            double dY = -dXR * cosAvg + dYR * sinAvg;

            pose = new Pose(pose.x + dX, pose.y + dY, heading);

            double vBL = bl.getVelocity();
            double vFL = fl.getVelocity();
            double vFR = fr.getVelocity();
            double vBR = br.getVelocity();

            double vXR = 0.25 * (-vBL + vFL - vFR + vBR) / (TICKS_PER_INCH * STRAFE_RATIO);
            double vYR = 0.25 * (vBL + vFL + vFR + vBR) / TICKS_PER_INCH;
            double vA = 0.25 * (-vBL - vFL + vFR + vBR) / TICKS_PER_RAD;

            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vX = vXR * sin + vYR * cos;
            double vY = -vXR * cos + vYR * sin;

            velocity = new Pose(vX, vY, vA);
        }
    }


}
