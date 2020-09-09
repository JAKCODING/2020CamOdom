package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.EnforcersLocalizer;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.Gyroscope;

import java.util.Arrays;

@TeleOp(name = "localizer test")
public class LocalizerTest extends OpMode {

    EnforcersLocalizer localizer;
    Pose2d pose = new Pose2d();
    LynxModule controlHub;

    DcMotor fLeft, bLeft, fRight, bRight;
    DcMotorEx encX, encY, encR;

    public void init() {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        fLeft = hardwareMap.dcMotor.get("fLeft");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");

        encX = hardwareMap.get(DcMotorEx.class, "bRight");
        encR = hardwareMap.get(DcMotorEx.class,"bLeft");
        encY = hardwareMap.get(DcMotorEx.class,"fRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new EnforcersLocalizer(hardwareMap);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        fLeft.setPower(gamepad1.left_stick_y);
        fRight.setPower(-gamepad1.left_stick_y);
        bLeft.setPower(-gamepad1.left_stick_y);
        bRight.setPower(gamepad1.left_stick_y);

        localizer.update();
        pose = localizer.getPoseEstimate();
        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addLine("RawX1: " + localizer.bRead.getMotors()[0] + ", Raw X2: " + localizer.bRead.getMotors()[1] + ", Raw Y: " + localizer.bRead.getMotors()[2]);
        telemetry.addLine("X1: " + localizer.encoderTicksToInches((int)localizer.bRead.getMotors()[0]) + ", X2: " + localizer.encoderTicksToInches((int)localizer.bRead.getMotors()[1]) + ", Y: " + localizer.encoderTicksToInches((int)localizer.bRead.getMotors()[2]));
        telemetry.addLine("X1: " + localizer.encoderTicksToInches(encX.getCurrentPosition()) + ", X2: " + localizer.encoderTicksToInches(encR.getCurrentPosition()) + ", Y: " + localizer.encoderTicksToInches(encY.getCurrentPosition()));
        telemetry.addData("Vel: ", Arrays.toString(localizer.getWheelVelocities().toArray()));
        telemetry.addData("Velocity: ", localizer.getPoseVelocity());
        telemetry.addData("Ang: ", Math.toDegrees(pose.getHeading()));
    }
}
