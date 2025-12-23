package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.xdrive.XDrive;
import org.firstinspires.ftc.teamcode.xdrive.XDriveAuto;

import java.util.List;

public abstract class DriveBaseAuto {


    public DriveBaseV2 bot;
    public double headingOffsetRadians = 0;
    public IMU imu;
    public DcMotorEx bl, fl, br, fr;
    private int ticksBL = 0, ticksFL = 0, ticksFR = 0, ticksBR = 0;
    public Pose pose = new Pose(0,0,0);


    private void setHeadingRadians(double headingRadians) {
        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        headingOffsetRadians = AngleUnit.normalizeRadians(headingRadians - rawHeading);
    }

    public void setPose (double x, double y, double hDegrees){
        double hRadians = Math.toRadians(hDegrees);
        setHeadingRadians(hRadians);
        ticksBL = bl.getCurrentPosition();
        ticksFL = fl.getCurrentPosition();
        ticksFR = fr.getCurrentPosition();
        ticksBR = br.getCurrentPosition();
        pose = new Pose(x, y, hRadians);
    }

    public void init(HardwareMap hwMap) {

        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Get the DC Motors
        bl = hwMap.get(DcMotorEx.class, "back_left_motor");
        fl = hwMap.get(DcMotorEx.class, "front_left_motor");
        fr = hwMap.get(DcMotorEx.class, "front_right_motor");
        br = hwMap.get(DcMotorEx.class, "back_right_motor");

        // Set zero power behavior to BRAKE for all motors (so robot stops quickly)
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Set Direction to Reverse for the correct motors
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

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





    }
}




