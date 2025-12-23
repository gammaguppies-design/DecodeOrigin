package org.firstinspires.ftc.teamcode.deep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.i2c.MaxSonarModified;
import org.firstinspires.ftc.teamcode.xdrive.XDrive;

public class DeepBot extends XDrive {

    public DcMotorEx armMotor;
    public DcMotorEx slideMotor;
    public DcMotorEx winchMotor;
    public Servo wristServo;
    public Servo clawServo;
    public Servo yawServo;
    public DistanceSensor distLeft;
    public  DistanceSensor distBack;
//    public MaxSonarModified sonicLeft;

    public static final double ARM_TICKS_PER_DEGREE = 18.9;  // Ticks on arm motor per degree elevation
    public static final double SLIDE_TICKS_PER_INCH = 114.04;  // Ticks on slide motor per inch of travel
    public static final double MIN_ARM_DEGREES = -47.7;   // Smallest (most negative) allowed arm angle
    public static final double MAX_ARM_DEGREES = 75;    // Largest allowed arm angle
    public static final double MAX_SLIDE_LENGTH = 48;   // Maximum allowed slide length (arm motor shaft to end of slide)
    public static final double SAFE_SLIDE_LENGTH = 33;  // Maximum slide length that will fit within 42" bounding box for all arm angles
    public static final double PAYLOAD_DIST_OFFSET = 1.75;  // Max distance from end of slide to end of payload, inches
    public static final double SAFE_ARM_ANGLE = 63;     // Min arm angle that respects 42" bounding box for all slide lengths
    public static final double SLIDE_BASE_LENGTH = 14.25;   // Arm shaft to end of slide when fully retracted
    public static final double ARM_FULCRUM_HIEGHT = 7.5;
    public static final double SLIDE_STAGE_LENGTH = 13.25;  // Length of individual slide stage
    public static final double SLIDE_STAGE_THROW = 9.5; // Greatest distance moved by individual slide stage relative to the stage below
    public static final double[] SLIDE_STAGE_MASS = {1.0, 1.0, 1.0, 1.0, 0.0};  // Relative mass of individual slide stages (last entry is payload)
    public static final double PAYLOAD_MASS_OFFSET = 3.0;   // Distance from end of slide to center of mass of payload

    public static final double TORQUE_CONSTANT = 0.1;   // Feedforward constant for arm elevation control
    public static final double INERTIA_CONSTANT = 0.001;    // Proportionate constant for arm elevation control

    private double targetSlideLength = SLIDE_BASE_LENGTH;
    private double targetArmAngle = MIN_ARM_DEGREES;

    public static final double CLAW_OPEN = 0.20;  // was 0.16
    
    public static final double  CLAW_WIDE_OPEN = 0.3;
    public static final double CLAW_CLOSED = 0.03;
    //change 0.03 to 0.04
    public static final double YAW_STRAIGHT = 0.414;
    public static final double YAW_LEFT = 0.744;
    public static final double YAW_RIGHT = 0.081;



    public void init(HardwareMap hwMap, boolean autonomous) {
        super.init(hwMap);

        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");
        slideMotor = hwMap.get(DcMotorEx.class, "slide_motor");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (autonomous) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            armMotor.setTargetPosition(0);
            slideMotor.setTargetPosition(0);
            distLeft = hwMap.get(DistanceSensor.class, "left_dist");
////            distBack = hwMap.get(DistanceSensor.class, "back_dist");
//            sonicLeft = hwMap.get(MaxSonarModified.class, "Sonic");

        } else {
            armMotor.getCurrentPosition();
            slideMotor.getCurrentPosition();
            int armPosition = armMotor.getCurrentPosition();
            int slidePosition = slideMotor.getCurrentPosition();
            armMotor.setTargetPosition(armPosition);
            slideMotor.setTargetPosition(slidePosition);
            targetArmAngle = armDegreesFromTicks(armPosition);
            targetSlideLength = slideInchesFromTicks(slidePosition);
        }

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setTargetPositionTolerance(10);
        slideMotor.setTargetPositionTolerance(10);

        slideMotor.setPower(1);
        if (!autonomous) armMotor.setPower(1);

        winchMotor = hwMap.get(DcMotorEx.class,"winchMotor");
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setTargetPosition(0);
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotor.setPower(1);

        wristServo = hwMap.get(Servo.class, "wristServo");
        clawServo = hwMap.get(Servo.class, "clawServo");
        yawServo = hwMap.get(Servo.class, "yawServo");

    }

    public void init(HardwareMap hwMap){
        init(hwMap, true);
    }

    public void resetArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setTargetPosition(0);
        slideMotor.setTargetPosition(0);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        targetArmAngle = armDegreesFromTicks(0);
        targetSlideLength = slideInchesFromTicks(0);
    }

    /*
     * Set target slide length to requested value, but constrained to respect 42" boundary
     */
    public double setTargetSlideLengthSafe(double inches) {
        inches = Range.clip(inches, SLIDE_BASE_LENGTH, MAX_SLIDE_LENGTH);
        if (targetArmAngle < SAFE_ARM_ANGLE) {
            inches = Math.min(inches, SAFE_SLIDE_LENGTH - 2);
        }
        targetSlideLength = inches;
        return targetSlideLength;
    }

    public double setTargetSlideLengthUnSafe(double inches) {
        inches = Math.min(inches, SAFE_SLIDE_LENGTH);
        targetSlideLength = inches;
        return targetSlideLength;
    }

    public double getTargetSlideLength() {
        return targetSlideLength;
    }

    /*
     * Set target arm angle to requested value, but simultaneously constrain the target
     * slide length to respect 42" boundary
//     */
    public double setTargetArmAngleSafe(double degrees) {
        targetArmAngle = Range.clip(degrees, MIN_ARM_DEGREES, MAX_ARM_DEGREES);
        if (targetArmAngle < SAFE_ARM_ANGLE) {
            targetSlideLength = Math.min(targetSlideLength, SAFE_SLIDE_LENGTH - 2);
        }
        return targetArmAngle;
    }

    public double setTargetArmAngleUnSafe(double degrees) {
        targetArmAngle = Math.min(degrees, 75);
        return targetArmAngle;
    }

    public double getTargetArmAngle() {
        return targetArmAngle;
    }

    public void seekArmTargets(double degrees, double inches) {
        int targetArmTicks = armTicksFromDegrees(degrees);

//        if (targetArmTicks > armMotor.getTargetPosition()){
//            raisingArm = true;
//        } else if (targetArmTicks < armMotor.getTargetPosition()){
//            raisingArm = false;
//        }

        int targetSlideTicks = slideTicksFromInches(inches);
        armMotor.setTargetPosition(targetArmTicks);
        slideMotor.setTargetPosition(targetSlideTicks);

        armMotor.setPower(1.0);

        slideMotor.setPower(1.0);
    }

    public void updateArm() {
        seekArmTargets(targetArmAngle, targetSlideLength);
    }


    /*
     * Update arm and slide motors using a combination of feedforward and proportionate control
     * for arm angle, and RUN_TO_POSITION mode for slide length.
     *
     * For arm angle control, the feedforward component of motor power is based on the torque being
     * applied to the arm by gravity. The TORQUE_CONSTANT should be adjusted to that the arm holds
     * its current elevation when powered only with the feedforward control. The proportionate
     * component of motor power is based on the moment of inertia of the arm.
     */
    public void updateArmNew() {
        int armTicks = armMotor.getCurrentPosition();
        int slideTicks = slideMotor.getCurrentPosition();
        double armDegrees = armDegreesFromTicks(armTicks);  // Current elevation angle of arm
        double slideInches = slideInchesFromTicks(slideTicks);  // Current length of slide

        /*
         * Current position of the center of each slide stage relative to the arm motor shaft.
         * The final element of this array is the position of payload center of mass relative to the
         * arm motor shaft.
         */
        double[] x = {SLIDE_BASE_LENGTH - SLIDE_STAGE_LENGTH / 2, 0, 0, 0, 0};
        if (slideInches < SLIDE_BASE_LENGTH + SLIDE_STAGE_THROW) {
            x[1] = slideInches - SLIDE_STAGE_LENGTH / 2;
            x[2] = x[1];
            x[3] = x[2];
        } else if (slideInches < SLIDE_BASE_LENGTH + 2 * SLIDE_STAGE_THROW) {
            x[1] = x[0] + SLIDE_STAGE_THROW;
            x[2] = slideInches - SLIDE_STAGE_LENGTH / 2;
            x[3] = x[2];
        } else {
            x[1] = x[0] + SLIDE_STAGE_THROW;
            x[2] = x[0] + 2 * SLIDE_STAGE_THROW;
            x[3] = slideInches - SLIDE_BASE_LENGTH;
        }

        x[4] = slideInches + PAYLOAD_MASS_OFFSET;

        /*
         * Compute torque being applied by gravity to the arm, and arm rotational moment of inertia,
         * by adding components for each individual stage (as well as the payload)
         */

        double gravityTorque = 0;
        double inertiaMoment = 0;

        for (int i = 0; i < 5; i++) {
            gravityTorque += SLIDE_STAGE_MASS[i] * x[i];
            inertiaMoment += SLIDE_STAGE_MASS[i] * x[i] * x[i];
            if (i < 4) {
                inertiaMoment += SLIDE_STAGE_MASS[i] * SLIDE_STAGE_LENGTH * SLIDE_STAGE_LENGTH / 12;
            }
        }

        gravityTorque *= Math.cos(Math.toRadians(armDegrees));

        /*
         * Use temporary target angle and length to avoid exceeding 42" boundary during travel
         * from current state to actual target state.
         */
        double tempTargetAngle = targetArmAngle;
        double tempTargetLength = targetSlideLength;

        if (armDegrees > SAFE_ARM_ANGLE) {
            if (targetArmAngle < SAFE_ARM_ANGLE && slideInches > SAFE_SLIDE_LENGTH) {
                tempTargetAngle = armDegrees;
            }
        } else {
            tempTargetLength = Math.min(SAFE_SLIDE_LENGTH, targetSlideLength);
        }

        double armPower = TORQUE_CONSTANT * gravityTorque + INERTIA_CONSTANT * inertiaMoment * (tempTargetAngle - armDegrees);
        armMotor.setPower(armPower);
        slideMotor.setTargetPosition(slideTicksFromInches(tempTargetLength));
        slideMotor.setPower(1);
    }

    public int armTicksFromDegrees(double degrees) {
        return (int) ((degrees - MIN_ARM_DEGREES) * ARM_TICKS_PER_DEGREE);
    }

    public int slideTicksFromInches(double inches) {
        return (int) ((inches - SLIDE_BASE_LENGTH) * SLIDE_TICKS_PER_INCH);
    }

    public double armDegreesFromTicks(int ticks) {
        return MIN_ARM_DEGREES + ticks / ARM_TICKS_PER_DEGREE;
    }

    public double slideInchesFromTicks(int ticks) {
        return SLIDE_BASE_LENGTH + ticks / SLIDE_TICKS_PER_INCH;
    }

    public double getArmAngle() {
        return armDegreesFromTicks(armMotor.getCurrentPosition());
    }

    public double getSlideInches() {
        return slideInchesFromTicks(slideMotor.getCurrentPosition());
    }

    public void setArmDegrees(double degrees) {
        int ticks = armTicksFromDegrees(degrees);
        armMotor.getCurrentPosition();
        armMotor.setTargetPosition(ticks);
        armMotor.setPower(1);
    }

    public boolean isArmBusy() {
        return armMotor.isBusy();
    }

    public void setSlideInches(double inches) {
        int ticks = slideTicksFromInches(inches);
        slideMotor.getCurrentPosition();
        slideMotor.setTargetPosition(ticks);
        slideMotor.setPower(1);
    }

    public boolean isSlideBusy() {
        return slideMotor.isBusy();
    }

    public void setWristPosition(double pos) {
        wristServo.setPosition(pos);
    }

    public void setClawPosition(double pos) {
        clawServo.setPosition(pos);
    }

    public void closeClaw() {
        setClawPosition(CLAW_CLOSED);
    }

    public void openClaw() {
        setClawPosition(CLAW_OPEN);
    }

    public void  setYawPosition(double pos){yawServo.setPosition(pos);}

    public void  setYawStraight(){setYawPosition(YAW_STRAIGHT);}
    public void setYawRight(){setYawPosition(YAW_RIGHT);}
    public void setYawLeft(){setYawPosition(YAW_LEFT);}


    public double getLeftDistance(){
        return distLeft.getDistance(DistanceUnit.INCH);
    }
    public  double getBackDistance(){
        return  distBack.getDistance(DistanceUnit.INCH);
    }

    public void setWinchPower(double power){
        winchMotor.setPower(power);
    }

    public void setWinchPosition(int pos){
        winchMotor.setTargetPosition(pos);
        winchMotor.setPower(1);
    }


}



