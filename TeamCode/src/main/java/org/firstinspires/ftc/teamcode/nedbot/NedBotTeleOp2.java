/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.nedbot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Toggle;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name="NedBot Tele2", group="Robot")
public class NedBotTeleOp2 extends LinearOpMode {

    boolean  yState = false;


    /* Declare OpMode members. */
    public DcMotor leftFrontDrive = null; //the left front drivetrain motor
    public DcMotor leftBackDrive = null; //the left back drivetrain motor
    public DcMotor rightFrontDrive = null; //the right front drivetrain motor
    public DcMotor rightBackDrive = null; //the right back drivetrain motor

    //Arm
    public DcMotor rightIntake = null; //the right intake motor
    public DcMotor leftIntake = null; //the left intake motor


  // intake smart servos
    public Servo ramp = null; //the active intake servo
    public Servo leftIntakeRotate = null;
    public Servo rightIntakeRotate = null;
    public Servo Flicker = null;

    // intake CRservo
    public CRServo flyWheel = null;
    public CRServo Roller = null;
    public CRServo Roller2 = null;


    Toggle toggleA2 = new Toggle(() -> gamepad2.a);
    Toggle toggleY1 = new Toggle(() -> gamepad1.y);
    Toggle toggley2 = new Toggle(()-> gamepad2.y);
    Toggle toggleX2 = new Toggle(() -> gamepad2.x);
    Toggle toggleb2 = new Toggle(() -> gamepad2.b);

    enum slowMode {Fast, Slow}
    enum IntakeState {OFF, INTAKE, DEPOSIT, INTAKE2}

    IntakeState intakeState = IntakeState.OFF;

    slowMode Slowmode = slowMode.Fast;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 248 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;





      // drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive"); //the left front drive motor
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");//the left back Dive motor
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");//the right front drive motor
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");//the right back drive motor

//        intake motors
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");//the right intake motor
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");//the left intake motor

        // intake CRservos
        flyWheel = hardwareMap.get(CRServo.class, "flyWheel");
        Roller = hardwareMap.get(CRServo.class,"Roller");
        Roller2 = hardwareMap.get(CRServo.class,"Roller2");

        // intake smart Servos
        ramp = hardwareMap.get(Servo.class, "ramp");
        leftIntakeRotate = hardwareMap.get(Servo.class, "left_Intake_Rotater");
        rightIntakeRotate = hardwareMap.get(Servo.class, "right_Intake_Rotater");
        Flicker = hardwareMap.get(Servo.class,"Flicker");



       leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Arm
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);



        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
//        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);




        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();


        /* Wait for the game driver to press play */
        waitForStart();


        /* Run until the driver presses stop */
        while (opModeIsActive()) {


            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_y;


            /* Here we "mix" the input channels together to find the power to apply to each motor.
            The both motors need to be set to a mix of how much you're retesting the robot move
            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
            the right and left motors need to move in opposite directions. So we will add rotate to
            forward for the left motor, and subtract rotate from forward for the right motor. */

            left = forward + rotate;
            right = -forward + rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            double maxStored = max;
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftFrontDrive.setPower(left);
            leftBackDrive.setPower(left);
            rightFrontDrive.setPower(right);
            rightBackDrive.setPower(right);

            if(yState == true) {
                leftFrontDrive.setPower(left / 2);
                leftBackDrive.setPower(left / 2);
                rightFrontDrive.setPower(right / 2);
                rightBackDrive.setPower(right / 2);
            }
            else{
                leftFrontDrive.setPower(left);
                leftBackDrive.setPower(left);
                rightFrontDrive.setPower(right);
                rightBackDrive.setPower(right);
            }
            boolean ToggledA2 = toggleA2.update();
            boolean ToggledX2 = toggleX2.update();
            boolean Toggledb2 = toggleb2.update();
            boolean ToggledY1 = toggleY1.update();
            boolean Toggledy2 = toggley2.update();


            if (ToggledY1) {
                if (yState == false) {
                    yState = true;
                    telemetry.addData("slowMode", yState);
                    telemetry.update();
                } else if (yState == true) {
                    yState = false;
                    telemetry.addData("slowMode", yState);
                    telemetry.update();
                }
            }



//                     controller 2

            switch (intakeState) {
                case OFF:
                    if (ToggledA2) {
                        rightIntake.setPower(0);
                        leftIntake.setPower(0);
                        flyWheel.setPower(0);
                        leftIntakeRotate.setPosition(0.5);
                        rightIntakeRotate.setPosition(0.5);
                        ramp.setPosition(0);
                        Roller.setPower(0);
                        Roller2.setPower(0);
                    }
                case INTAKE:
                    if (Toggledb2) {
                        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                        flyWheel.setPower(1);
                        rightIntake.setPower(0.2);
                        leftIntake.setPower(0.2);
                        leftIntakeRotate.setPosition(0.5);
                        rightIntakeRotate.setPosition(0.5);
                        ramp.setPosition(0.2);
                        Flicker.setPosition(3);
                        Roller.setPower(1);
                        Roller2.setPower(-1);

                    }
                case DEPOSIT:
                    if (ToggledX2) {
                        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                        rightIntake.setPower(0.9);
                        leftIntake.setPower(0.9);
                        Roller.setPower(-1);
                        Roller2.setPower(1);
                        leftIntakeRotate.setPosition(0.3);
                        rightIntakeRotate.setPosition(0.7);
                        ramp.setPosition(16);
                        sleep(900);
                        flyWheel.setPower(-1);
                        Flicker.setPosition(-3);
                    }
                case INTAKE2:
                    if (Toggledy2){
                        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                        Roller.setPower(-1);
                        Roller2.setPower(1);
                        leftIntakeRotate.setPosition(0.5);
                        rightIntakeRotate.setPosition(0.5);
                        flyWheel.setPower(1.5);
                        rightIntake.setPower(0.2);
                        leftIntake.setPower(0.2);
                        ramp.setPosition(0);
                        Flicker.setPosition(3);
                    }

            }



        }
    }
//    public void drive(double y, double x, double rx) {
//        double frontLeftPower  = y + x + rx;
//        double backLeftPower   = y - x + rx;
//        double frontRightPower = y - x - rx;
//        double backRightPower  = y + x - rx;
//
//        // Normalize power levels
//        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
//                Math.max(Math.abs(backLeftPower),
//                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));
//
//        leftFrontDrive.setPower(frontLeftPower / max);
//        leftBackDrive.setPower(backLeftPower / max);
//        rightBackDrive.setPower(frontRightPower / max);
//        rightFrontDrive.setPower(backRightPower / max);
//    }
}
