package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="TestT265")
public class TestT265 extends OpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    T265Camera slamra;

    double x=0,y=0,ang=0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());
        Transform2d cameraToRobot = new Transform2d(new Translation2d(0, 0), new Rotation2d(0));

        slamra = new T265Camera(cameraToRobot, 1, hardwareMap.appContext);
        slamra.setPose(startingPose);
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();

        try {
            x = update.pose.getTranslation().getX();
            y = update.pose.getTranslation().getY();
            ang = update.pose.getRotation().getDegrees();
        }
        catch (Exception ignored) {

        }

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Angle", ang);
    }

    @Override
    public void stop() {
        slamra.stop();
    }
}
