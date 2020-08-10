package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class LocalizerTest extends OpMode {

    DcMotor encX, encY, encR;
    LynxModule controlHub;
    BulkRead bRead;
    SampleMecanumDrive drive;
    int stepCounter = 0;

    public void init() {

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        encX = hardwareMap.dcMotor.get("encX");
        encY = hardwareMap.dcMotor.get("encY");
        encR = hardwareMap.dcMotor.get("encR");
        bRead = new BulkRead(controlHub, encX, encY, encR, new int[]{0});
        drive = new SampleMecanumDrive(hardwareMap);
    }

    public void loop() {

        switch(stepCounter) {

            case 0:
                Trajectory traj = drive.trajectoryBuilder(new Pose2d(-45,-45))
                        .forward(10)
                        .build();

        }

    }

    public Pose2d currentPose = new Pose2d();

    public class EnforcersThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {

        public EnforcersThreeWheelLocalizer(@NotNull List<Pose2d> wheelPoses) {
            super(wheelPoses);
        }
        public List<Double> getWheelPositions() {
            return Arrays.asList(
                    bRead.getMotors()[0],
                    bRead.getMotors()[1],
                    bRead.getMotors()[2]
            );
        }



    }
}
