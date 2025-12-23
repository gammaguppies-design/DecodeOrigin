package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enh;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhImpl;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
@Disabled
public class TestBNO055Enh extends LinearOpMode {

    BNO055EnhImpl imu;

    BNO055Enh.AxesMap aMap = BNO055Enh.AxesMap.XYZ;
    BNO055Enh.AxesSign aSign = BNO055Enh.AxesSign.PPP;

    Toggle toggleA1 = new Toggle(()->gamepad1.a);
    Toggle toggleB1 = new Toggle(()->gamepad1.b);
    Toggle toggleX1 = new Toggle(()->gamepad1.x);

    public void runOpMode(){

        imu = hardwareMap.get(BNO055EnhImpl.class, "imu");

        while (opModeInInit()){
            telemetry.addData("Press A for Map, B for Sign","");
            if (toggleA1.update()){
                aMap = BNO055Enh.AxesMap.values()[(aMap.ordinal()+1)%6];
            }
            if (toggleB1.update()){
                aSign = BNO055Enh.AxesSign.values()[(aSign.ordinal()+1)%8];
            }
            telemetry.addData("AxesMap", aMap);
            telemetry.addData("AxesSign", aSign);
            telemetry.update();
        }

        BNO055Enh.Parameters parameters = new BNO055Enh.Parameters();
        parameters.axesMap = aMap;
        parameters.axesSign = aSign;

        imu.initialize(parameters);
        ElapsedTime delay = new ElapsedTime();
        while (opModeIsActive() && delay.milliseconds()<3000) continue;

        while (opModeIsActive()){

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("ZYX angles","%.1f  %.1f  %.1f", orientation.firstAngle,
                    orientation.secondAngle, orientation.thirdAngle);

            telemetry.update();
        }

    }
}
