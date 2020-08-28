package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.EnforcersLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.EnforcersLocalizer.WHEEL_RADIUS;

@TeleOp(name = "SimpleLocalizerTest")
public class SimpleLocalizerTest extends OpMode {

    ThreeWheelTracking localizer;

    DcMotor fLeft, fRight, bLeft, bRight;

    @Override
    public void init() {
        localizer = new ThreeWheelTracking(hardwareMap);

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        localizer.update();
        Pose2d pose = localizer.getPoseEstimate();
        List<Double> wheelPositions = localizer.getWheelPositions();

        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addData("Ang: ", Math.toDegrees(pose.getHeading()));

        telemetry.addLine();
        telemetry.addData("XFront: ", wheelPositions.get(0));
        telemetry.addData("XBack: ", wheelPositions.get(1));
        telemetry.addData("YBack: ", wheelPositions.get(2));

        localizer.update();

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        fLeft.setPower((leftY - leftX + rightX)/3);
        fRight.setPower((leftY + leftX - rightX)/3);
        bLeft.setPower((leftY + leftX + rightX)/3);
        bRight.setPower((leftY - leftX - rightX)/3);
    }

    @Override
    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}

class ThreeWheelTracking extends ThreeTrackingWheelLocalizer {

    private final static double Y_OFFSET = 6.5;
    private final static double X_OFFSET = -2.5;

    DcMotorEx xFront, xBack, yBack;

    public ThreeWheelTracking(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, Y_OFFSET, Math.toRadians(90)),
                new Pose2d(0, -Y_OFFSET, Math.toRadians(90)),
                new Pose2d(X_OFFSET, -Y_OFFSET, Math.toRadians(0))
        ));

        xFront = hardwareMap.get(DcMotorEx.class, "bLeft");
        xBack = hardwareMap.get(DcMotorEx.class, "bRight");
        yBack = hardwareMap.get(DcMotorEx.class, "fRight");

        xBack.setDirection(DcMotorSimple.Direction.REVERSE);
        yBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(xFront.getCurrentPosition()),
                encoderTicksToInches(xBack.getCurrentPosition()),
                encoderTicksToInches(yBack.getCurrentPosition())
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(xFront.getVelocity()),
                encoderTicksToInches(xBack.getVelocity()),
                encoderTicksToInches(yBack.getVelocity())
        );
    }

    private static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * (ticks / TICKS_PER_REV);
    }
}

/*

    */
/**
         * Get the position using the tracking camera
         * @return Position in inches and degrees
         *//*

    public Pose2d getCamPosition() {
        return new Pose2d(poseX, poseY, poseAng);
    }

    */
/**
     * Get the velocity using the tracking camera
     * @return Velocity in inches per second and degrees per second
     *//*

    public Pose2d getCamVelocity() {
        return new Pose2d(velX, velY, velAng);
    }

    */
/**
     * Get the position using wheeled odometry
     * @return Position in inches and radians
     *//*

    public Pose2d getOdomPosition() {
        return getPoseEstimate();
    }

    */
/**
     * Get the velocity using wheeled odometry
     * @return Velocity in inches per second and radians per second
     *//*

    public Pose2d getOdomVelocity() {
        return getOdomPosition().minus(lastPose).div((System.currentTimeMillis() - lastTime)/1000);
    }

    @Override
    public void update() {
        super.update();
        velocity = getOdomVelocity();
        lastPose = getOdomPosition();
        lastTime = System.currentTimeMillis();

        if (isStarted) {
            slamra.sendOdometry(velocity.getY(), -velocity.getX());
            slamra.update();

            poseX = slamra.getX();
            poseY = slamra.getY();
            poseAng = slamra.getAng();

            velX = slamra.getVelX();
            velY = slamra.getVelY();
            velAng = slamra.getVelAng();
        }
    }*/
