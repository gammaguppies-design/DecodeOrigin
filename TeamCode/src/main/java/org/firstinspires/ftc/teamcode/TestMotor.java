package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestMotor extends LinearOpMode {

    DcMotorEx motor;

    public void runOpMode(){

        // Get a motor object

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while(opModeIsActive()){

            // Check left joystick on gamepad 1

            double power = -gamepad1.left_stick_y;

            motor.setPower(power);


        }

        motor.setPower(0);

    }
}
