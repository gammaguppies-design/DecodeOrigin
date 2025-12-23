package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
@Disabled
public class TestIMU extends LinearOpMode {

    IMU imu;

    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    Toggle toggleA1 = new Toggle(()->gamepad1.a);
    Toggle toggleB1 = new Toggle(()->gamepad1.b);
    Toggle toggleX1 = new Toggle(()->gamepad1.x);

    public void runOpMode(){

        imu = hardwareMap.get(IMU.class, "imu");

        while (opModeInInit()){
            telemetry.addData("Press A for LogoFD, B for UsbFD","");
            if (toggleA1.update()){
                logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.values()[(logoFacingDirection.ordinal()+1)%6];
            }
            if (toggleB1.update()){
                usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.values()[(usbFacingDirection.ordinal()+1)%6];
            }
            if (logoFacingDirection.ordinal()/2 == usbFacingDirection.ordinal()/2){
                telemetry.addData("ILLEGAL COMBINATION OF LOGO AND USB DIRECTION", "");
            }
            telemetry.addData("LogoFacingDirection", logoFacingDirection);
            telemetry.addData("USBFacingDirection", usbFacingDirection);
            telemetry.update();
            toggleX1.update();
        }

        if (logoFacingDirection.ordinal()/2 == usbFacingDirection.ordinal()/2){
            while (opModeIsActive()){
                telemetry.addData("ILLEGAL COMBO. START OVER","");
                telemetry.update();
                return;
            }
        }

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)));

        ElapsedTime et = new ElapsedTime();
        while (et.seconds() < 0.3 && !Thread.currentThread().isInterrupted()) continue;

        double rawHeadingAtReset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingOffsetDegrees = -rawHeadingAtReset;

        while (opModeIsActive()){
            telemetry.addData("Press X to reset yaw", "");
            if (toggleX1.update()){
                rawHeadingAtReset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                headingOffsetDegrees = -rawHeadingAtReset;
            }

            Quaternion quaternion = imu.getRobotOrientationAsQuaternion();
            YawPitchRollAngles yprAngles = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Quaternion","%.3f  %.3f  %.3f  %.3f", quaternion.w,
                    quaternion.x, quaternion.y, quaternion.z);

            VectorF qVector = new VectorF(quaternion.x, quaternion.y, quaternion.z);

            VectorF qAxis = qVector.normalized3D();
            telemetry.addData("Q Axis", "%.3f  %.3f  %.3f", qAxis.get(0), qAxis.get(1), qAxis.get(2));

            double qAngleDegrees = AngleUnit.normalizeDegrees(2*Math.toDegrees(Math.atan2(qVector.magnitude(), quaternion.w)));
            telemetry.addData("Q Angle", qAngleDegrees);

            telemetry.addData("YPR Angles", "%.1f  %.1f  %.1f",
                    AngleUnit.normalizeDegrees(headingOffsetDegrees + yprAngles.getYaw(AngleUnit.DEGREES)),
                    yprAngles.getPitch(AngleUnit.DEGREES), yprAngles.getRoll(AngleUnit.DEGREES));

            telemetry.addData("Raw Heading at reset", rawHeadingAtReset);

            telemetry.update();
        }

    }
}
