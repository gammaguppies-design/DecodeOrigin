package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
@Disabled
public class TestArm  extends LinearOpMode {
	
	// Testing
	// And testing again
	// and once more
	// ok again

    DcMotorEx armMotor;
    DcMotorEx slideMotor;

    public void runOpMode(){
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slide_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            armMotor.setPower(gamepad1.left_stick_y);
            slideMotor.setPower(-gamepad1.right_stick_y);
            telemetry.addData("armTicks", armMotor.getCurrentPosition());
            telemetry.addData("slideTicks", slideMotor.getCurrentPosition());
            telemetry.update();
            continue;
        }


    }
}
