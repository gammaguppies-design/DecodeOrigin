package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
@Disabled
public class TestMaxBotics extends LinearOpMode {

    MaxSonarI2CXL sonic;

    public void runOpMode(){
        sonic = hardwareMap.get(MaxSonarI2CXL.class, "Sonic");

        waitForStart();

        while (opModeIsActive()){
            double inches = sonic.getDistanceAsync(50, DistanceUnit.INCH);
            telemetry.addData("Sonic inches", inches);
            telemetry.update();
        }
    }
}
