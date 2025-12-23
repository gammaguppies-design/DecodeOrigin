package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class XDriveTele extends LinearOpMode {

    protected XDrive bot = null;

    private Toggle toggleA1 = new Toggle(()-> gamepad1.a);
    private  Toggle toggleB1 = new Toggle(()-> gamepad1.b);
    private Toggle toggleX1 = new Toggle(()-> gamepad1.x);
    boolean slowMode = true;
    double speedDivider = 4;
    double turnDivider = 4;
    boolean fieldcentric = true;

    public void setBot(XDrive bot){
        this.bot = bot;
    }

    public void runOpMode(){
        bot = new XDrive();
        bot.init(hardwareMap);
//        bot.setPose(0, 0, 0);

        waitForStart();

        toggleB1.update();

        while (opModeIsActive()){
            oneDriveCycle();
            telemetry.addData("ticks", "bl %d  fl %d  fr %d  br %d",
                    bot.bl.getCurrentPosition(), bot.fl.getCurrentPosition(),
                    bot.fr.getCurrentPosition(), bot.br.getCurrentPosition());
            telemetry.update();
        }
    }

    protected void oneDriveCycle(){
        if (toggleA1.update()){
            slowMode = !slowMode;
            if (slowMode){
                speedDivider = 4;
                turnDivider = 4;
            }else {
                speedDivider = 1.75;
                turnDivider = 3;
            }
        }
        if (toggleB1.update()){
            fieldcentric = !fieldcentric;
        }
        bot.updateOdometry();
        Pose pose = bot.getPose();
        Pose vel = bot.getVelocity();
        if (toggleX1.update()){
            double newHeading = 90;
//            bot.setPose(pose.x, pose.y, newHeading);
            pose = bot.getPose();
        }
        double gpx = gamepad1.left_stick_x;
        double gpy = -gamepad1.left_stick_y;
        double gpa = gamepad1.left_trigger - gamepad1.right_trigger;
        double pxr, pyr, pa;
        pa = gpa/turnDivider;
        if (fieldcentric){
            double px = gpx/speedDivider;
            double py = gpy/speedDivider;
            double cos = Math.cos(pose.h);
            double sin = Math.sin(pose.h);
            pxr = px * sin - py * cos;
            pyr = px * cos + py * sin;
        }else {
            pxr = gpx / speedDivider;
            pyr = gpy / speedDivider;
        }
        bot.setDrivePower(pxr, pyr, pa);
        telemetry.addData("SlowMode", slowMode);
        telemetry.addData("fieldCentric", fieldcentric);
        telemetry.addData("Pos","x %.1f y %.1f h %.1f", pose.x, pose.y,
                Math.toDegrees(pose.h));
    }

}
