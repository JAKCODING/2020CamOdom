/*
package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Headless;
import org.firstinspires.ftc.teamcode.drive.Tank;
import org.firstinspires.ftc.teamcode.test.SuperAuto;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.Gyroscope;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.EnforcersLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.EnforcersLocalizer.WHEEL_RADIUS;

@TeleOp(name="twowheeltest")
public class TwoWheelTest extends OpMode {

    TwoWheelLocalizer localizer;
    Pose2d pose = new Pose2d();
    BulkRead bRead;
    Gyroscope gyro;
    LynxModule controlHub;

    DcMotor fLeft, bLeft, fRight, bRight, encX, encY, encR;

    int step = 0;

    public void init() {

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        fLeft = hardwareMap.dcMotor.get("fLeft");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");

        encX = hardwareMap.dcMotor.get("bLeft");
        encR = hardwareMap.dcMotor.get("bRight");
        encY = hardwareMap.dcMotor.get("fRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        bRead = new BulkRead(controlHub, encX, encR, encY);
        gyro = new Gyroscope(telemetry, hardwareMap);

        localizer = new TwoWheelLocalizer(bRead, gyro);

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
        fRight.setPower(gamepad1.left_stick_y);
        bLeft.setPower(gamepad1.left_stick_y);
        bRight.setPower(gamepad1.left_stick_y);

        localizer.update();
        pose = localizer.getPoseEstimate();
        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addLine("EncX: " + bRead.getMotors()[0] + ", enc Y: " + bRead.getMotors()[1]);
        telemetry.addData("Ang: ", pose.getHeading());
    }

}

class TwoWheelLocalizer implements Localizer {

    BulkRead bRead;
    Gyroscope gyro;

    double poseX=0, poseY=0, poseAng=0;

    TwoWheelLocalizer(BulkRead bRead, Gyroscope gyro) {
        this.bRead = bRead;
        this.gyro = gyro;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(poseX, poseY, poseAng);
    }//

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        double[] mPose = bRead.getMotors();
        poseX = pose2d.getX();
        poseY = pose2d.getY();
        poseAng = pose2d.getHeading();
    }

    @Override
    public void update() {

        double[] mPose = bRead.getMotors();
        poseX = encoderTicksToInches(mPose[0]);
        poseY = -encoderTicksToInches(mPose[1]);
        poseAng = gyro.getYaw();

    }
}
*/
