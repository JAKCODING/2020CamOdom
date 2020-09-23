package org.firstinspires.ftc.teamcode.drive;
import android.printservice.PrintService;
import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.T265.T265;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Config
public class EnforcersLocalizer extends ThreeTrackingWheelLocalizer {

    //Hardware
    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    LynxModule lMod;
    public BulkRead bRead;
    T265 slamra;

    //Positional values (tuned) for the odometry wheels
    public static double LATERAL_DISTANCE = /*12.6*/13.4; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.3; // in; offset of the lateral wheel

    //Values for odometry wheels and encoders
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .74d; // in
    public static double TICKS_PER_INCH = TICKS_PER_REV/(2 * Math.PI * WHEEL_RADIUS);
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //Multipliers for each odometry wheel
    public static double rMultiplier = 1.022247126d;
    public static double xMultiplier = 1.027365986d;
    public static double yMultiplier = 1.014739085d;

    //Stored position and velocity values for T265
    double poseX=0, poseY=0, poseAng=0;
    double velX=0, velY=0, velAng=0;
    Pose2d velocity = new Pose2d();

    public EnforcersLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, LATERAL_DISTANCE /2, Math.toRadians(90)) // front
        ));

        lMod = hardwareMap.get(LynxModule.class, "Control Hub");

        leftEncoder = hardwareMap.get(DcMotorEx.class, "bRight");
        rightEncoder = hardwareMap.get(DcMotorEx.class,"bLeft");
        frontEncoder = hardwareMap.get(DcMotorEx.class,"fRight");

        bRead = new BulkRead(lMod, leftEncoder, rightEncoder, frontEncoder);
        //slamra = new T265(hardwareMap, 0, -4.6, 7.75, 0);

    }

    /**
     * Starts the T265 camera
     */
    public void start() {
        slamra.start();
    }

    /**
     * Stops the T265 camera
     */
    public void stop() {
        slamra.stop();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double[] vel = bRead.getMotors();
        return Arrays.asList(
                encoderTicksToInches(vel[0]) * xMultiplier,
                encoderTicksToInches(vel[1]) * rMultiplier,
                -encoderTicksToInches(vel[2]) * yMultiplier
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        double[] vel = bRead.getMVelocity();
        return Arrays.asList(
                encoderTicksToInches(vel[0]) * xMultiplier,
                encoderTicksToInches(vel[1]) * rMultiplier,
                -encoderTicksToInches(vel[2]) * yMultiplier
        );
    }

    @Override
    public void update() {
        super.update();
        velocity = getPoseVelocity();

        slamra.sendOdometry(-velocity.getY(), velocity.getX());
        slamra.update();

        poseX = slamra.getX();
        poseY = slamra.getY();
        poseAng = slamra.getAng();

        velX = slamra.getVelX();
        velY = slamra.getVelY();
        velAng = slamra.getVelAng();
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
     * Converts encoder ticks to inches travelled
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}