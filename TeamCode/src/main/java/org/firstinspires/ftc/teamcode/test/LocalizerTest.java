package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.Pose2dConvert;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class LocalizerTest extends OpMode {

    DcMotor encX, encY, encR;
    LynxModule controlHub;
    BulkRead bRead;
    T265Camera cam;
    SampleMecanumDrive drive;

    public void init() {

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        cam = new T265Camera(new Transform2d(), 0, hardwareMap.appContext);
        encX = hardwareMap.dcMotor.get("encX");
        encY = hardwareMap.dcMotor.get("encY");
        encR = hardwareMap.dcMotor.get("encR");
        bRead = new BulkRead(controlHub, encX, encY, encR, new int[]{0});
        drive = new SampleMecanumDrive(hardwareMap);

    }

    public void loop() {

    }

    public Pose2d currentPose = new Pose2d();

    public class EnforcersThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {

        Pose2d camPose;

        public void setCamPose(Pose2d camPose) {
            this.camPose = camPose;
        }

        public EnforcersThreeWheelLocalizer(@NotNull List<Pose2d> wheelPoses, Pose2d camPose) {
            super(wheelPoses);
            this.camPose = camPose;
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
