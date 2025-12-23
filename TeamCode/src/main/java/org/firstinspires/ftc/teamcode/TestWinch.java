package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class TestWinch extends LinearOpMode {

    DcMotorEx winch;


    public void runOpMode(){
        winch = hardwareMap.get(DcMotorEx.class, "winchMotor");
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()){

            double power = -gamepad1.left_stick_y;
            winch.setPower(power);
            telemetry.addData("power", power);
            telemetry.addData("ticks", winch.getCurrentPosition());
            telemetry.update();
        }



    }

}
