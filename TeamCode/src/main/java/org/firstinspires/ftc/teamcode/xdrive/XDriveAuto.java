package org.firstinspires.ftc.teamcode.xdrive;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.deep.DeepBot;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.QuinticSpline2D;

import java.util.LinkedList;


public abstract class XDriveAuto extends LinearOpMode {

    protected XDrive bot;

    protected MotionProfile slow = new MotionProfile(10, 20, 10);
    protected MotionProfile medium = new MotionProfile(15, 30, 15);
    protected MotionProfile fast = new MotionProfile(25, 60, 25);
    protected MotionProfile superFast = new MotionProfile(30, 60, 30);


    public Localizer localizer;
    public void setBot(XDrive bot) {
        this.bot = bot;
    }


    public void addPoseToTelemetry(String caption, Pose pose) {
        telemetry.addData(caption, "x = %.1f  y = %.1f  h = %.1f",
                pose.x, pose.y, Math.toDegrees(pose.h));
    }

    public void driveTo(MotionProfile mProfile, double targetX, double targetY,
                        double targetHeadingDegrees, double tolerance) {
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        Pose startPose = bot.getPose();

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();
            VectorF fromStart = new VectorF((float) (pose.x - startPose.x), (float) (pose.y - startPose.y));
            VectorF toTarget = new VectorF((float) (targetX - pose.x), (float) (targetY - pose.y));
            double d1 = fromStart.magnitude();
            double d2 = toTarget.magnitude();

            if (d2 < tolerance) {
                break;
            }

            VectorF dir = toTarget.multiplied(1.0f / (float) d2);

            double v1 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d1);
            double v2 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d2);
            double v = Math.min(v1, v2);
            v = Math.min(v, mProfile.vMax);

            double vX = v * dir.get(0);
            double vY = v * dir.get(1);

            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vXR = vX * sin - vY * cos;
            double vYR = vX * cos + vY * sin;

            double vA = 2.0 * AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            bot.setDriveSpeed(vXR, vYR, vA);
        }

        bot.setDrivePower(0, 0, 0);
    }

    public void SetsPose(double x, double y, double hDegrees){
        localizer.setPose(x, y, hDegrees);
    }

    public void driveLine(MotionProfile mProf, Pose p0, Pose p1, double tolerance) {
        VectorF lineDir = new VectorF((float) (p1.x - p0.x), (float) (p1.y - p0.y));
        float totalDist = lineDir.magnitude();
        lineDir = lineDir.multiplied(1.0f / totalDist);

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();

            if (Math.sqrt((p1.x - pose.x) * (p1.x - pose.x) + (p1.y - pose.y) * (p1.y - pose.y)) < tolerance) {
                break;
            }

            VectorF vPose0 = new VectorF((float) (pose.x - p0.x), (float) (pose.y - p0.y));
            float d0 = vPose0.dotProduct(lineDir);  // how far we've gone
            VectorF vPose1 = new VectorF((float) (p1.x - pose.x), (float) (p1.y - pose.y));
            float d1 = vPose1.dotProduct(lineDir);  // how far we have to go
            float err = lineDir.get(0) * vPose0.get(1) - lineDir.get(1) * vPose0.get(0); // distance from pose to line
            VectorF vErr = new VectorF(-err * lineDir.get(1), err * lineDir.get(0));    // Linear error vector
            float speed0 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d0) * mProf.accel);
            float speed1 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d1) * mProf.accel);
            float speed = (float) Math.min(mProf.vMax, Math.min(speed0, speed1));
            VectorF baseVel = lineDir.multiplied(speed * Math.signum(d1));
            VectorF corrVel = vErr.multiplied(-0.2f * speed);
            VectorF velRobot = fieldToBot(baseVel.added(corrVel), pose.h);
            double va = 4.0 * AngleUnit.normalizeRadians(p1.h - pose.h);
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }


    public void driveLine(MotionProfile mProf, VectorF p0, VectorF p1, double targetHeadingDegrees, double tolerance) {
        VectorF lineDir = p1.subtracted(p0);
        float totalDist = lineDir.magnitude();
        lineDir = lineDir.multiplied(1.0f / totalDist);

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();

            if (Math.sqrt((p1.get(0) - pose.x) * (p1.get(0) - pose.x) + (p1.get(1) - pose.y) * (p1.get(1) - pose.y)) < tolerance) {
                break;
            }

            VectorF vPose0 = new VectorF((float) (pose.x - p0.get(0)), (float) (pose.y - p0.get(1)));
            float d0 = vPose0.dotProduct(lineDir);  // how far we've gone
            VectorF vPose1 = new VectorF((float) (p1.get(0) - pose.x), (float) (p1.get(1) - pose.y));
            float d1 = vPose1.dotProduct(lineDir);  // how far we have to go
            float err = lineDir.get(0) * vPose0.get(1) - lineDir.get(1) * vPose0.get(0); // distance from pose to line
            VectorF vErr = new VectorF(-err * lineDir.get(1), err * lineDir.get(0));    // Linear error vector
            float speed0 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d0) * mProf.accel);
            float speed1 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d1) * mProf.accel);
            float speed = (float) Math.min(mProf.vMax, Math.min(speed0, speed1));
            VectorF baseVel = lineDir.multiplied(speed * Math.signum(d1));
            VectorF corrVel = vErr.multiplied(-0.2f * speed);
            VectorF velRobot = fieldToBot(baseVel.added(corrVel), pose.h);
            double va = 4.0 * AngleUnit.normalizeRadians(Math.toRadians(targetHeadingDegrees) - pose.h);
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }

    public void driveLine(MotionProfile mProf, Pose p0, Pose p1, double tolerance,
                          Runnable runnable) {
        VectorF lineDir = new VectorF((float) (p1.x - p0.x), (float) (p1.y - p0.y));
        float totalDist = lineDir.magnitude();
        lineDir = lineDir.multiplied(1.0f / totalDist);

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();
            runnable.run();

            if (Math.sqrt((p1.x - pose.x) * (p1.x - pose.x) + (p1.y - pose.y) * (p1.y - pose.y)) < tolerance) {
                break;
            }

            VectorF vPose0 = new VectorF((float) (pose.x - p0.x), (float) (pose.y - p0.y));
            float d0 = vPose0.dotProduct(lineDir);  // how far we've gone
            VectorF vPose1 = new VectorF((float) (p1.x - pose.x), (float) (p1.y - pose.y));
            float d1 = vPose1.dotProduct(lineDir);  // how far we have to go
            float err = lineDir.get(0) * vPose0.get(1) - lineDir.get(1) * vPose0.get(0); // distance from pose to line
            VectorF vErr = new VectorF(-err * lineDir.get(1), err * lineDir.get(0));    // Linear error vector
            float speed0 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d0) * mProf.accel);
            float speed1 = (float) Math.sqrt(mProf.vMin * mProf.vMin + 2.0 * Math.abs(d1) * mProf.accel);
            float speed = (float) Math.min(mProf.vMax, Math.min(speed0, speed1));
            VectorF baseVel = lineDir.multiplied(speed * Math.signum(d1));
            VectorF corrVel = vErr.multiplied(-0.2f * speed);
            VectorF velRobot = fieldToBot(baseVel.added(corrVel), pose.h);
            double va = 4.0 * AngleUnit.normalizeRadians(p1.h - pose.h);
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }

    public void jogTo(MotionProfile mProf, double targetX, double targetY, double targetHeadingDegrees,
                      double tolerance, boolean rightFirst) {
        Pose pose = bot.getPose();
        VectorF yr = new VectorF((float) Math.cos(pose.h), (float) Math.sin(pose.h));
        VectorF xr = new VectorF((float) Math.sin(pose.h), -(float) Math.cos(pose.h));
        VectorF ur = xr.added(yr).multiplied(1.0f / (float) Math.sqrt(2));
        VectorF vr = yr.subtracted(xr).multiplied(1.0f / (float) Math.sqrt(2));
        VectorF start = new VectorF((float) pose.x, (float) pose.y);
        VectorF target = new VectorF((float) targetX, (float) targetY);
        VectorF diff = target.subtracted(start);
        VectorF pIntermed;
        if (rightFirst) {
            pIntermed = start.added(ur.multiplied(diff.dotProduct(ur)));
        } else {
            pIntermed = start.added(vr.multiplied(diff.dotProduct(vr)));
        }
        driveTo(mProf, pIntermed.get(0), pIntermed.get(1), targetHeadingDegrees, tolerance);
        driveTo(mProf, targetX, targetY, targetHeadingDegrees, tolerance);
    }

    public void turnTo(double targetHeadingDegrees, double vaMaxDegrees, double coeff, double toleranceDegrees) {
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        double vaMaxRadians = Math.toRadians(vaMaxDegrees);
        double toleranceRadians = Math.toRadians(toleranceDegrees);

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();
            Pose velocity = bot.getVelocity();
            double offset = AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            if (Math.abs(offset) < toleranceRadians && Math.abs(velocity.h) < 0.5) {
                break;
            }

            double va = coeff * offset;
            if (Math.abs(va) > vaMaxRadians) {
                va = vaMaxRadians * Math.signum(va);
            }

            bot.setDriveSpeed(0, 0, va);
        }

        bot.setDrivePower(0, 0, 0);
    }


    public void turnTo(double targetHeadingDegrees, double vaMaxDegrees, double coeff,
                       boolean clockwise, double toleranceDegrees) {
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        double vaMaxRadians = Math.toRadians(vaMaxDegrees);
        double toleranceRadians = Math.toRadians(toleranceDegrees);

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pose pose = bot.getPose();
            Pose velocity = bot.getVelocity();
            double offset = AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            if (Math.abs(offset) < toleranceRadians && Math.abs(velocity.h) < 0.5) {
                break;
            }

            if (clockwise && offset > 0.5) {
                offset -= 2 * Math.PI;
            } else if (!clockwise && offset < -0.5) {
                offset += 2 * Math.PI;
            }

            double va = coeff * offset;
            if (Math.abs(va) > vaMaxRadians) {
                va = vaMaxRadians * Math.signum(va);
            }

            bot.setDriveSpeed(0, 0, va);
        }

        bot.setDrivePower(0, 0, 0);
    }


    /**
     * Drive quintic spline from current position and heading to target position and heading.
     *
     * @param mProf             Min and max speed, acceleration
     * @param endX              target X coordinate
     * @param endY              target Y coordinate
     * @param endHeadingDegrees Target final TRAVEL DIRECTION
     * @param tolerance         target final coordinate tolerance, inches
     * @param reverse           If true, drive in reverse
     */
    public void splineTo(MotionProfile mProf, double endX, double endY,
                         double endHeadingDegrees, double tolerance, boolean reverse) {
        bot.updateOdometry();
        double startHeadingRadians = bot.getPose().h;
        if (reverse) {
            startHeadingRadians = AngleUnit.normalizeRadians(startHeadingRadians + Math.PI);
        }
        double endHeadingRadians = Math.toRadians(endHeadingDegrees);
        VectorF endVec = new VectorF((float) endX, (float) endY);
        QuinticSpline2D spline = new QuinticSpline2D(bot.getPose().x, bot.getPose().y,
                endX, endY, startHeadingRadians, endHeadingRadians);
        double totalLength = spline.getLength();
        VectorF prevTargVec = spline.p(0);
        double s = 0;
        double d0 = 0;
        double absDistToEnd = 0;
        double headingError = 0;
        VectorF vel;
        VectorF velRobot;
        double va = 0;

        while (opModeIsActive()) {
            bot.updateOdometry();
            VectorF poseVec = new VectorF((float) bot.getPose().x, (float) bot.getPose().y);

            // Find closest point on spline to current pose
            s = spline.findClosestS(poseVec.get(0), poseVec.get(1), s);

            // Distance (crow flies) between bot pose and end point
            absDistToEnd = poseVec.subtracted(endVec).magnitude();

            // If at or beyond end of spline or within tolerance, break
            if (s >= 0.9999999 || absDistToEnd < tolerance) break;

            // Where bot should be now along spline
            VectorF currTargVec = spline.p(s);

            // Update distance traveled (d0) along spline so far, and find
            // remaining distance along spline (d1) to end
            VectorF deltaTargVec = currTargVec.subtracted(prevTargVec);
            d0 += deltaTargVec.magnitude();
            if (d0 > totalLength) d0 = totalLength;
            double d1 = totalLength - d0;

            // targErr is vector from current bot pose to where bot should be on spline
            VectorF targErr = currTargVec.subtracted(poseVec);

            // travelDir is unit vector giving current direction of travel along spline
            VectorF travelDir = spline.d1(s);
            travelDir = travelDir.multiplied(1.0f / travelDir.magnitude());

            // Compute nominal travel speed, v
            double v0 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            double v1 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            double v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            // Compute travel velocity, including nominal velocity plus correction
            vel = travelDir.multiplied((float) v).added(targErr.multiplied(8.0f));

            // Convert vel from field coordinates to robot coordinates
            velRobot = fieldToBot(vel, bot.getPose().h);

            // Compute target heading at current location on spline, and heading rate of change
            double targetHeading = spline.getHeading(s);
            if (reverse) {
                targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
            }
            double targetHeadingRate = spline.getHeadingRateOfChange(s, v);

            // Compute current heading error
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);

            // Compute heading speed, including nominal rate of change plus correction
            va = targetHeadingRate + 4.0 * headingError;

            // Set robot speed
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);

            prevTargVec = currTargVec;
        }

        /*
         * We have either reached the end of the spline or reached end point within tolerance.
         * In case tolerance not reached, allow 1 second period of adjustment
         */

        ElapsedTime et = new ElapsedTime();

        double targetHeading = endHeadingRadians;
        if (reverse) {
            targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
        }

        while (opModeIsActive() && et.seconds() < 1) {
            bot.updateOdometry();
            VectorF vecToEnd = endVec.subtracted(new VectorF((float) bot.getPose().x, (float) bot.getPose().y));
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);
            if (vecToEnd.magnitude() < tolerance && Math.abs(headingError) < Math.toRadians(3))
                break;
            vel = vecToEnd.multiplied(8);
            velRobot = fieldToBot(vel, bot.getPose().h);
            va = 4.0 * headingError;
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }


    /**
     * Drive quintic spline from current position and heading to target position and heading.
     *
     * @param mProf             Min and max speed, acceleration
     * @param endX              target X coordinate
     * @param endY              target Y coordinate
     * @param endHeadingDegrees Target final TRAVEL DIRECTION
     * @param tolerance         target final coordinate tolerance, inches
     * @param reverse           If true, drive in reverse
     * @param alpha             Higher alpha -> less turning at the beginning, more at the middle of path
     * @param beta              Higher beta -> less turning at the end, more at the middle of path
     */
    public void splineTo(MotionProfile mProf, double endX, double endY, double endHeadingDegrees,
                         double tolerance, boolean reverse, double alpha, double beta) {
        bot.updateOdometry();
        double startHeadingRadians = bot.getPose().h;
        if (reverse) {
            startHeadingRadians = AngleUnit.normalizeRadians(startHeadingRadians + Math.PI);
        }
        double endHeadingRadians = Math.toRadians(endHeadingDegrees);
        VectorF endVec = new VectorF((float) endX, (float) endY);
        QuinticSpline2D spline = new QuinticSpline2D(bot.getPose().x, bot.getPose().y,
                endX, endY, startHeadingRadians, endHeadingRadians, alpha, beta);
        double totalLength = spline.getLength();
        VectorF prevTargVec = spline.p(0);
        double s = 0;
        double d0 = 0;
        double absDistToEnd = 0;
        double headingError = 0;
        VectorF vel;
        VectorF velRobot;
        double va = 0;

        while (opModeIsActive()) {
            bot.updateOdometry();
            VectorF poseVec = new VectorF((float) bot.getPose().x, (float) bot.getPose().y);

            // Find closest point on spline to current pose
            s = spline.findClosestS(poseVec.get(0), poseVec.get(1), s);

            // Distance (crow flies) between bot pose and end point
            absDistToEnd = poseVec.subtracted(endVec).magnitude();

            // If at or beyond end of spline or within tolerance, break
            if (s >= 0.9999999 || absDistToEnd < tolerance) break;

            // Where bot should be now along spline
            VectorF currTargVec = spline.p(s);

            // Update distance traveled (d0) along spline so far, and find
            // remaining distance along spline (d1) to end
            VectorF deltaTargVec = currTargVec.subtracted(prevTargVec);
            d0 += deltaTargVec.magnitude();
            if (d0 > totalLength) d0 = totalLength;
            double d1 = totalLength - d0;

            // targErr is vector from current bot pose to where bot should be on spline
            VectorF targErr = currTargVec.subtracted(poseVec);

            // travelDir is unit vector giving current direction of travel along spline
            VectorF travelDir = spline.d1(s);
            travelDir = travelDir.multiplied(1.0f / travelDir.magnitude());

            // Compute nominal travel speed, v
            double v0 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            double v1 = Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            double v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            // Compute travel velocity, including nominal velocity plus correction
            vel = travelDir.multiplied((float) v).added(targErr.multiplied(8.0f));

            // Convert vel from field coordinates to robot coordinates
            velRobot = fieldToBot(vel, bot.getPose().h);

            // Compute target heading at current location on spline, and heading rate of change
            double targetHeading = spline.getHeading(s);
            if (reverse) {
                targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
            }
            double targetHeadingRate = spline.getHeadingRateOfChange(s, v);

            // Compute current heading error
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);

            // Compute heading speed, including nominal rate of change plus correction
            va = targetHeadingRate + 4.0 * headingError;

            // Set robot speed
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);

            prevTargVec = currTargVec;
        }

        /*
         * We have either reached the end of the spline or reached end point within tolerance.
         * In case tolerance not reached, allow 1 second period of adjustment
         */

        ElapsedTime et = new ElapsedTime();

        double targetHeading = endHeadingRadians;
        if (reverse) {
            targetHeading = AngleUnit.normalizeRadians(targetHeading + Math.PI);
        }

        while (opModeIsActive() && et.seconds() < 1) {
            bot.updateOdometry();
            VectorF vecToEnd = endVec.subtracted(new VectorF((float) bot.getPose().x, (float) bot.getPose().y));
            headingError = AngleUnit.normalizeRadians(targetHeading - bot.getPose().h);
            if (vecToEnd.magnitude() < tolerance && Math.abs(headingError) < Math.toRadians(3))
                break;
            vel = vecToEnd.multiplied(8);
            velRobot = fieldToBot(vel, bot.getPose().h);
            va = 4.0 * headingError;
            bot.setDriveSpeed(velRobot.get(0), velRobot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }


    public void driveUntilStopped(double speed, double directionDegrees,
                                  Predicate<Pose> pred) {
        Pose pose = bot.getPose();
        double heading = pose.h;
        double directionRadians = Math.toRadians(directionDegrees);
        VectorF vField = new VectorF((float) (speed * Math.cos(directionRadians)),
                (float) (speed * Math.sin(directionRadians)));
        VectorF vRobot = fieldToBot(vField, heading);
        bot.setDriveSpeed(vRobot.get(0), vRobot.get(1), 0);

        ProgressChecker pc = new ProgressChecker(500);
        ElapsedTime et = new ElapsedTime();

        bot.otosLoc.setPose(pose.x, pose.y, Math.toDegrees(pose.h));

        while (opModeIsActive()) {
            bot.updateOdometry();
            Pair<Double, Pose> progress = pc.check();
            if (et.milliseconds() > 500 && progress != null
                    && pred.test(progress.second)) {
                break;
            }
        }

        bot.setDriveSpeed(0, 0, 0);

    }

    protected VectorF fieldToBot(VectorF vField, double heading) {
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vField.get(0) * sinTheta - vField.get(1) * cosTheta, vField.get(0) * cosTheta + vField.get(1) * sinTheta);
    }

    protected VectorF botToField(VectorF vBot, double heading) {
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vBot.get(0) * sinTheta + vBot.get(1) * cosTheta, -vBot.get(0) * cosTheta + vBot.get(1) * sinTheta);
    }


    public class ProgressChecker {
        private LinkedList<Pair<Double, Pose>> queue = new LinkedList<>();
        private double interval;

        public ProgressChecker(double millis) {
            interval = millis;
            queue.add(new Pair<>(System.nanoTime() / 1000000.0, bot.getPose()));
        }

        public Pair<Double, Pose> check() {
            double currentMillis = System.nanoTime() / 1000000.0;
            Pose currentPose = bot.otosLoc.getPose();

            Pair<Double, Pose> queuePeek = queue.peek();
            if (queuePeek == null) {
                queue.add(new Pair<>(currentMillis, currentPose));
                return null;
            }
            double queuePeekMillis = queuePeek.first.doubleValue();
            double elapsedMillis = currentMillis - queuePeekMillis;
            if (elapsedMillis < interval) {
                queue.add(new Pair<>(currentMillis, currentPose));
                return null;
            }
            Pose queuePeekPose = queuePeek.second;
            Pose progress = new Pose(currentPose.x - queuePeekPose.x, currentPose.y - queuePeekPose.y,
                    currentPose.h - queuePeekPose.h);

            while (elapsedMillis > interval) {
                queue.remove();
                queuePeek = queue.peek();
                if (queuePeek == null) {
                    queue.add(new Pair<>(currentMillis, currentPose));
                    return null;
                }
                queuePeekMillis = queuePeek.first.doubleValue();
                elapsedMillis = currentMillis - queuePeekMillis;
                progress = new Pose(currentPose.x - queuePeekPose.x, currentPose.y - queuePeekPose.y,
                        currentPose.h - queuePeekPose.h);
            }

            queue.add(new Pair<Double, Pose>(currentMillis, currentPose));
            return new Pair<Double, Pose>(elapsedMillis, progress);


        }
    }
}




