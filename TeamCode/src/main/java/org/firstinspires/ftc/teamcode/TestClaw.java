package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.deep.DeepBot;

@TeleOp
public class TestClaw extends LinearOpMode {

    DeepBot bot = new DeepBot();
    double clawPos =  0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setClawPosition(clawPos);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                clawPos += 0.0002;
            } else if (gamepad1.dpad_down){
                clawPos -= 0.0002;
            }

            bot.setClawPosition(clawPos);
            telemetry.addData("clawServo", clawPos);
            telemetry.update();
        }
    }
}
