package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(name = "mitusskidster.java (Blocks to Java)")
public class mitusskidster extends LinearOpMode {

        private DcMotor rightFrontDrive;
        private DcMotor rightBackDrive;
        private DcMotor leftFrontDrive;
        private DcMotor leftBackDrive;

        Toggle toggleb1 = new Toggle(() -> gamepad2.b);
        boolean ToggledRb2 = toggleb1.update();
        /**
         * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
         * Comment Blocks show where to place Ini   tialization code (runs once, after touching the
         * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
         * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
         * Stopped).
         */
        @Override
        public void runOpMode() {
            // TODO: Enter the type for variable named item
            boolean item;


            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");

            waitForStart();
            if (opModeIsActive()) {

                while (opModeIsActive()) {
//                    item += "forword";
                    if (ToggledRb2) {
                        rightFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                    }
                    if (gamepad1.dpad_up) {
                        rightFrontDrive.setPower(0.7);
                        rightBackDrive.setPower(0.7);
                        leftFrontDrive.setPower(-0.7);
                        leftBackDrive.setPower(0.7);
                    }
//                    item += "Reverse";
                    if (gamepad1.dpad_down) {
                        rightFrontDrive.setPower(-0.7);
                        rightBackDrive.setPower(-0.7);
                        leftFrontDrive.setPower(0.7);
                        leftBackDrive.setPower(-0.7);
                    }
//                    item += "Right turn";
                    if (gamepad1.dpad_right) {
                        rightFrontDrive.setPower(-0.1);
                        rightBackDrive.setPower(-0.1);
                        leftFrontDrive.setPower(-0.8);
                        leftBackDrive.setPower(0.8);
                    }
//                    item += "left turn";
                    if (gamepad1.dpad_left) {
                        rightFrontDrive.setPower(0.8);
                        rightBackDrive.setPower(0.8);
                        leftFrontDrive.setPower(0.25);
                        leftBackDrive.setPower(-0.25);
                    }
                }
                telemetry.update();
            }
        }
    }

