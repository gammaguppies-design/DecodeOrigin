package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class TestNewArm extends LinearOpMode {

    DcMotorEx arm;
    DcMotorEx slide;

    public void runOpMode(){
        arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        slide = hardwareMap.get(DcMotorEx.class, "slide_motor");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        slide.setPower(1);

        waitForStart();

        while (opModeIsActive()){
            int armPos  = arm.getCurrentPosition();
            int slidePos = slide.getCurrentPosition();
            int armTarget = arm.getTargetPosition();
            int slideTarget = slide.getTargetPosition();


            telemetry.addData("Arm", "target = %d   actual = %d", armTarget, armPos);
            telemetry.addData("Slide", "target = %d   actual = %d", slideTarget, slidePos);
            telemetry.update();


            arm.setTargetPosition(armTarget +  (int) (20 * gamepad1.left_stick_y));
            slide.setTargetPosition(slideTarget -  (int) (20 * gamepad1.right_stick_y));
            arm.setPower(1);
            slide.setPower(1);

        }
    }
}
