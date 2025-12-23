package org.firstinspires.ftc.teamcode.StarterBotCode.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Toggle;
@TeleOp
public class GobilaStarterBot extends LinearOpMode {


    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;


    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();
    private Object launch;


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }


    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    Toggle toggleRB2 = new Toggle(() -> gamepad2.right_bumper);



   public void launch(boolean rightBumper) {

    }



    @Override
        public void runOpMode () throws InterruptedException {




        GobliudaTeleop.LaunchState launchState = GobliudaTeleop.LaunchState.IDLE;

                leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
                rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
                launcher = hardwareMap.get(DcMotorEx.class, "launcher");
                leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
                rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);

                launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);

                launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

                leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
                    while (opModeIsActive()) {
                        double x = gamepad1.right_stick_y;
                        double y = -gamepad1.left_stick_y;
                        double z = gamepad1.right_trigger;
                        double denominator = 0;


                        denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(z), 1.0);

                        if (gamepad2.a) {
                            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                        } else if (gamepad2.b) { // stop flywheel
                            launcher.setVelocity(STOP_SPEED);
                        }

                        launch(gamepad2.right_bumper);

                        telemetry.addData("State", launchState);
                        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                        telemetry.addData("motorSpeed", launcher.getVelocity());


                        boolean launch = toggleRB2.update();
                        boolean shotRequested = true;

                        switch (launchState) {
                            case IDLE:
                                if (shotRequested == true) {
                                    launchState = GobliudaTeleop.LaunchState.SPIN_UP;
                                }
                                break;
                            case SPIN_UP:
                                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                                    launchState = GobliudaTeleop.LaunchState.LAUNCH;
                                }
                                break;
                            case LAUNCH:
                                leftFeeder.setPower(FULL_SPEED);
                                rightFeeder.setPower(FULL_SPEED);
                                feederTimer.reset();
                                launchState = GobliudaTeleop.LaunchState.LAUNCHING;
                                break;
                            case LAUNCHING:
                                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                                    launchState = GobliudaTeleop.LaunchState.IDLE;
                                    leftFeeder.setPower(STOP_SPEED);
                                    rightFeeder.setPower(STOP_SPEED);
                                    break;
                                }


                        }

                    }
        }




}




