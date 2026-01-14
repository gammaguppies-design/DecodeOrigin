package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.xdrive.XDrive;
import org.firstinspires.ftc.teamcode.xdrive.XDriveAuto;



@Autonomous
public class DriveBaseTestAuto extends XDriveAuto {

    public void SetsPose(double x, double y, double hDegrees){
        localizer.setPose(x, y, hDegrees);
    }
    XDrive bot = new XDrive();


    public void setBot(XDrive bot){
        this.bot = bot;
    }

        @Override
        public void runOpMode() throws InterruptedException {
            bot.init(hardwareMap);
            super.setBot(bot);

            waitForStart();
            MotionProfile Fast = new MotionProfile(20, 30, 10);


            


            driveTo(Fast, 50, 40, 90, 1);



        }

    }

