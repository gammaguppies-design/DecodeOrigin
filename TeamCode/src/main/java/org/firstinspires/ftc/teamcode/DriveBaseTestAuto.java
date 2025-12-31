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
    protected XDrive bot;


    public void setBot(XDrive bot){
        this.bot = bot;
    }


        @Override
        public void runOpMode() throws InterruptedException {
            bot.init(hardwareMap);
            super.setBot(bot);

            waitForStart();


            MotionProfile slow = new MotionProfile(10, 20, 10);

            bot.setspose(48, 60, -90);

            driveTo(slow, 36, 54, 90, 1);
            turnTo(90, 10, 5, 1);

        }

    }

