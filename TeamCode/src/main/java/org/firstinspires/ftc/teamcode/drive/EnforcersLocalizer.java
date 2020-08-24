package org.firstinspires.ftc.teamcode.drive;
import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.T265.T265;
import org.firstinspires.ftc.teamcode.util.BulkRead;

import java.util.Arrays;
import java.util.List;

public class EnforcersLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = DriveConstants.TICKS_PER_REV;
    public static double WHEEL_RADIUS = DriveConstants.WHEEL_RADIUS; // in
    public static double GEAR_RATIO = DriveConstants.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7; // in; offset of the lateral wheel
    double poseX=0, poseY=0, poseAng=0;
    double velX=0, velY=0, velAng=0;

    int portL, portR, portF;

    boolean isStarted = false;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    BulkRead bRead;
    T265 slamra;

    public EnforcersLocalizer(HardwareMap hardwareMap/*, T265 slamra, BulkRead bRead*/) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("bLeft");
        rightEncoder = hardwareMap.dcMotor.get("fRight");
        frontEncoder = hardwareMap.dcMotor.get("bRight");

        portL = leftEncoder.getPortNumber(); portR = rightEncoder.getPortNumber(); portF = frontEncoder.getPortNumber();

        this.slamra = slamra;
        this.bRead = bRead;

    }

    public void start() {
        slamra.start();
        isStarted = true;
    }

    public void stop() {
        if (isStarted) {
            slamra.stop();
            isStarted = false;
        }
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @Override
    public void update() {
        super.update();
        if (isStarted) {
            slamra.update();

            poseX = slamra.getX();
            poseY = slamra.getY();
            poseAng = slamra.getAng();

            velX = slamra.getVelX();
            velY = slamra.getVelY();
            velAng = slamra.getVelAng();
        }
    }
}