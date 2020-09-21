package org.firstinspires.ftc.teamcode.T265;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.test.SuperAuto;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

@Autonomous(name="T265Localizer")
public class RoadRunnerT265 extends SuperAuto {

    T265Localizer localizer;

    Pose2d pose = new Pose2d();

    int step = 0;

    public void init_op() {
        localizer = new T265Localizer(slamra);
    }

    public void start() {
        localizer.start();
    }

    public void loop_op() {
        localizer.update();
        pose = localizer.getPoseEstimate();
        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addData("Ang: ", pose.getHeading());
    }

    public void stop() {
        localizer.stop();
    }
}

class T265Localizer implements Localizer {

    T265 slamra;

    double poseX=0, poseY=0, poseAng=0;
    double velX=0, velY=0, velAng=0;

    boolean isStarted = false;

    T265Localizer(T265 slamra) {
        this.slamra = slamra;
    }

    public void start() {
        slamra.start();
        isStarted = true;
    }

    public void stop() {
        if (isStarted) {
            slamra.stop();
            isStarted = false;
        }
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(poseX, poseY, poseAng);
    }//

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        if (isStarted) {
            slamra.setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        }
        poseX = pose2d.getX();
        poseY = pose2d.getY();
        poseAng = pose2d.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(velX, velY, velAng);
    }

    @Override
    public void update() {
        if (isStarted) {
            slamra.update();

            poseX = slamra.getX();
            poseY = slamra.getY();
            poseAng = slamra.getAng();

            velX = slamra.getVelX();
            velY = slamra.getVelY();
            velAng = slamra.getVelAng();
        }
    }
}
