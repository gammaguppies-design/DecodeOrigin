package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "HuskeyLens (Blocks to Java)")
public class HuskyLensTestOpmode extends LinearOpMode {






        private HuskyLens HuskyLens_HuskyLens;

        /**
         * This OpMode illustrates how to use the DFRobot HuskyLens.
         *
         * The HuskyLens is a Vision Sensor with a built-in object detection model. It can
         * detect a number of predefined objects and AprilTags in the 36h11 family, can
         * recognize colors, and can be trained to detect custom objects. See this website for
         * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
         *
         * This sample illustrates how to detect AprilTags, but can be used to detect other types
         * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
         * a name of "huskylens".
         */
        @Override
        public void runOpMode() {
            ElapsedTime myElapsedTime;

            HuskyLens_HuskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");

            // Put initialization blocks here.
            telemetry.addData(">>", HuskyLens_HuskyLens.knock() ? "Touch start to continue" : "Problem communicating with HuskyLens");
            HuskyLens_HuskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            HuskyLens_HuskyLens.knock();
            telemetry.update();
            myElapsedTime = new ElapsedTime();
            waitForStart();
            if (opModeIsActive()) {

                // Put run blocks here.
                while (opModeIsActive()) {
                    // Put loop blocks here.
                    if (myElapsedTime.seconds() >= 1) {
                        myElapsedTime.reset();
                        HuskyLens.Block[] blocks = HuskyLens_HuskyLens.blocks();

                        telemetry.addData("Block count", blocks.length);
                        if (blocks.length>0) {
                            telemetry.addData("My id", blocks[0].id);
                        }
                        for (int i=0; i< blocks.length; i++) {
                            telemetry.addData("Block", blocks[i].toString());
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

