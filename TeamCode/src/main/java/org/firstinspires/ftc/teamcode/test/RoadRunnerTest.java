package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Pose2dConvert;

@Autonomous(name = "RoadRunnerTest <3")
public class RoadRunnerTest extends LocalizerTest {

    int stepCounter = 0;
    Pose2dConvert p2c;

    public void init() {}

    public void loop() {
        switch (stepCounter) {

            case 0:
                Trajectory traj = drive.trajectoryBuilder(new Pose2d(-45, -45))
                        .forward(10)
                        .build();

        }

        cam.sendOdometry(bRead.getMVelocity()[0], bRead.getMVelocity()[2]);
        Pose2d camPose = p2c.convertToOdom(cam.getLastReceivedCameraUpdate().pose);
    }
}
