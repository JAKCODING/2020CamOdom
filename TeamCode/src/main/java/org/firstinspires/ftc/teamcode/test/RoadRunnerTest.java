package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.util.Pose2dConvert;

@Autonomous(name = "RoadRunnerTest <3")
public class RoadRunnerTest extends SuperAuto {

    int stepCounter = 0;
    Pose2dConvert p2c;
    T265Camera cam;

    public void init() {
        super.init();
        cam = new T265Camera(new Transform2d(), 0.05, hardwareMap.appContext);
    }

    public void loop() {
        cam.sendOdometry(bRead.getMVelocity()[0], bRead.getMVelocity()[2]);
        Pose2d camPose = p2c.convertToOdom(cam.getLastReceivedCameraUpdate().pose);

        switch (stepCounter) {

            case 0:
                Trajectory traj = drive.trajectoryBuilder(new Pose2d(-45, -45))
                        .forward(10)
                        .build();

        }

    }
}
