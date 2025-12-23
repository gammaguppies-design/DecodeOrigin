package org.firstinspires.ftc.teamcode.util;

public interface Localizer {
    Pose getPose();
    Pose getVelocity();
    void setPose(double x, double y, double headingDegrees);
    void update();
}
