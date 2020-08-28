package org.firstinspires.ftc.teamcode.drive;
import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.T265.T265;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

public class EnforcersLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = DriveConstants.TICKS_PER_REV;
    public static double WHEEL_RADIUS = DriveConstants.WHEEL_RADIUS; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.5; // in; offset of the lateral wheel
    double poseX=0, poseY=0, poseAng=0;
    double velX=0, velY=0, velAng=0;

    int portL, portR, portF;

    boolean isStarted = false;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    BulkRead bRead;
    T265 slamra;

    Pose2d velocity, lastPose;
    double lastTime;

    public EnforcersLocalizer(HardwareMap hardwareMap, /*T265 slamra,*/ BulkRead bRead) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("bRight");
        rightEncoder = hardwareMap.dcMotor.get("bLeft");
        frontEncoder = hardwareMap.dcMotor.get("fRight");

        portL = leftEncoder.getPortNumber(); portR = rightEncoder.getPortNumber(); portF = frontEncoder.getPortNumber();

        //this.slamra = slamra;
        this.bRead = bRead;

    }

    /*public void start() {
        slamra.start();
        isStarted = true;
    }

    public void stop() {
        if (isStarted) {
            slamra.stop();
            isStarted = false;
        }
    }*/

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double[] vel = bRead.getMotors();
        return Arrays.asList(
                encoderTicksToInches((int) vel[0]),
                encoderTicksToInches((int) vel[1]),
                encoderTicksToInches((int) vel[2])
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        double[] vel = bRead.getMVelocity();
        return Arrays.asList(
                encoderTicksToInches((int) vel[0]),
                encoderTicksToInches((int) vel[1]),
                encoderTicksToInches((int) vel[2])
        );
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
    }
    /**
     * Get the position using the tracking camera
     * @return Position in inches and degrees
     */

    public Pose2d getCamPosition() {
        return new Pose2d(poseX, poseY, poseAng);
    }


/**
 * Get the velocity using the tracking camera
 * @return Velocity in inches per second and degrees per second
 */

    public Pose2d getCamVelocity() {
        return new Pose2d(velX, velY, velAng);
    }


/**
 * Get the position using wheeled odometry
 * @return Position in inches and radians
 */

    public Pose2d getOdomPosition() {
        return getPoseEstimate();
    }


/**
 * Get the velocity using wheeled odometry
 * @return Velocity in inches per second and radians per second
 */

    public Pose2d getOdomVelocity() {
        return getOdomPosition().minus(lastPose).div((System.currentTimeMillis() - lastTime)/1000);
    }
}