/*Contains support for 400k i2c reads.*/
package org.firstinspires.ftc.teamcode.test;

import android.support.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
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

        encX = hardwareMap.dcMotor.get("bLeft");
        encR = hardwareMap.dcMotor.get("fRight");
        encY = hardwareMap.dcMotor.get("bRight");

        bRead = new BulkRead(controlHub, encX, encR, encY);
        drive = new SampleMecanumDrive(hardwareMap);

    }

    public void loop() {
    }

}
