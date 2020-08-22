package org.firstinspires.ftc.teamcode.drive;
import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class EnforcersLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = DriveConstants.TICKS_PER_REV;
    public static double WHEEL_RADIUS = DriveConstants.WHEEL_RADIUS; // in
    public static double GEAR_RATIO = DriveConstants.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public EnforcersLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("bLeft");
        rightEncoder = hardwareMap.dcMotor.get("fRight");
        frontEncoder = hardwareMap.dcMotor.get("bRight");

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
}