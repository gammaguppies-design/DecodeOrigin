
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.xdrive.XDrive;
import org.firstinspires.ftc.teamcode.xdrive.XDriveAuto;



@Autonomous
public class DriveBaseTestAuto extends XDriveAuto {

    public void setPose(double x, double y, double hDegrees){
        localizer.setPose(x, y, hDegrees);
    }
    XDrive bot = new XDrive();

    public Localizer localizer;
    public IMU imu;


    public void setBot(XDrive bot){
        this.bot = bot;
    }

        @Override
        public void runOpMode() throws InterruptedException {
            bot.init(hardwareMap);
            super.setBot(bot);

            waitForStart();
            MotionProfile Fast = new MotionProfile(20, 30, 10);

            Pose mypose = bot.getPose();

            bot.localizer.setPose(50, 60,90);

            driveTo(Fast, 20, 60, 90, 1);

            addPoseToTelemetry("pose", mypose);
            telemetry.update();

        }

    }

