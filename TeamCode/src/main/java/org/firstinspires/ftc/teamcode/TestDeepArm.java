package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class TestDeepArm extends LinearOpMode {

    DeepBot bot = new DeepBot();

    boolean ascending = false;

     Toggle toggleA1 = new Toggle(()-> gamepad1.a);

    public void runOpMode(){
        bot.init(hardwareMap);
        bot.setTargetArmAngleSafe(DeepBot.MIN_ARM_DEGREES);
        bot.setTargetSlideLengthSafe(DeepBot.SLIDE_BASE_LENGTH);
        bot.updateArm();
        waitForStart();
        while (opModeIsActive()){
            if (toggleA1.update()){
                ascending = !ascending;
            }
            double currentArmLength = bot.getSlideInches();
            double currentArmAngle = bot.getArmAngle();
            double targetArmLength = bot.getTargetSlideLength();
            double targetArmAngle = bot.getTargetArmAngle();
            bot.setTargetArmAngleSafe(targetArmAngle + gamepad1.left_stick_y * 0.5);
            bot.setTargetSlideLengthSafe(targetArmLength - gamepad1.right_stick_y * 0.2);
            bot.updateArm();

            telemetry.addData("targetangle", targetArmAngle);
            telemetry.addData("targetlength", targetArmLength);
            telemetry.addData("armdegrees", currentArmAngle);
            telemetry.addData("slideinches", currentArmLength);
            telemetry.addData("armTicks",bot.armMotor.getCurrentPosition());
            telemetry.addData("slideTicks", bot.slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
