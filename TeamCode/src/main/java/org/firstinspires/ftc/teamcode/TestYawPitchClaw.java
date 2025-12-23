package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestYawPitchClaw extends LinearOpMode {

    private Servo yawServo;
    private Servo pitchServo;
    private Servo clawServo;

    public static final double minYaw = 0;
    public static final double maxYaw = 1;
    public static final double minPitch = 0;
    public static final double maxPitch = 1;
    public static final double minClaw = 0;
    public static final double maxClaw = 1;

    public void runOpMode(){

        double yaw = (minYaw + maxYaw)/2;
        double pitch = (minPitch + maxPitch)/2;
        double claw = (minClaw + maxClaw)/2;

        yawServo = hardwareMap.get(Servo.class, "yawServo");
        pitchServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        yawServo.setPosition(yaw);
        pitchServo.setPosition(pitch);
        clawServo.setPosition(claw);

        waitForStart();

        while (opModeIsActive()){

            pitch += 0.0005 * gamepad1.left_stick_y;
            yaw += 0.0005 * gamepad1.right_stick_x;

            if (gamepad1.dpad_left){
                claw += 0.0005;
            } else if (gamepad1.dpad_right){
                claw -= 0.0005;
            }

            pitch = Math.max(minPitch, Math.min(maxPitch, pitch));
            yaw = Math.max(minYaw, Math.min(maxYaw, yaw));
            claw = Math.max(minClaw, Math.min(maxClaw, claw));

            yawServo.setPosition(yaw);
            pitchServo.setPosition(pitch);
            clawServo.setPosition(claw);

            telemetry.addData("Yaw (Rt Stick X)", yaw);
            telemetry.addData("Pitch (Lt stick Y", pitch);
            telemetry.addData("Claw (Dpad Lt/Rt", claw);
            telemetry.update();

        }

    }


}
