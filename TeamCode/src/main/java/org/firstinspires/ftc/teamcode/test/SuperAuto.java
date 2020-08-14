/*Contains support for 400k i2c reads.*/
package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.Pose2dConvert;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class SuperAuto extends OpMode {

    DcMotor encX, encY, encR;
    LynxModule controlHub;
    BulkRead bRead;
    SampleMecanumDrive drive;
    Pose2dConvert p2c;

    public void init() {

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

    }

    public void loop() {
        throw new Warning("This loop is on fire.");
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

            double camX = camPose.getX();
            double camY = camPose.getY();



            return Arrays.asList(
                    bRead.getMotors()[0],
                    bRead.getMotors()[1],
                    bRead.getMotors()[2]
            );
        }



    }
}
