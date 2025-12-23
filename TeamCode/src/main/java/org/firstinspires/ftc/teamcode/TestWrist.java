package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.deep.DeepBot;

@TeleOp
@Disabled
public class TestWrist extends LinearOpMode {

    DeepBot bot = new DeepBot();
    double wristPos =  0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setWristPosition(wristPos);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                wristPos += 0.0002;
            } else if (gamepad1.dpad_down){
                wristPos -= 0.0002;
            }

            bot.setWristPosition(wristPos);
            telemetry.addData("wrist", wristPos);
            telemetry.update();
        }
    }
}
