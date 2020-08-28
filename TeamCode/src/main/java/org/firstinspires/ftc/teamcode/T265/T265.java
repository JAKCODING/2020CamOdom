package org.firstinspires.ftc.teamcode.T265;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class T265 {

    //This is the actual camera object. It's name is a combination of VSLAM and camera
    T265Camera slamra;

    //These are the main position variables from the camera
    //Translation is the x and y position in meters
    //Rotation is the heading of the robot in radians
    //Velocity is the x, y, and angular velocity of the robot in m/s and rad/s
    //Confidence is the camera's confidence in its estimated position
    Translation2d translation = new Translation2d();
    Rotation2d rotation = new Rotation2d();
    ChassisSpeeds velocity = new ChassisSpeeds();
    T265Camera.PoseConfidence confidence = T265Camera.PoseConfidence.Failed;

    //Failed is used to make sure that if the camera wasn't initialized properly, it won't cause errors
    //UpdateIsNull is used to keep track of if a camera update didn't return values
    //The update is usually only null in the first update, but it's good to know if anything's gone wrong
    boolean failed = false;
    boolean updateIsNull = false;

    /**
     * This constructor initializes the T265 camera at a starting position of (0, 0) and an angle of 0.
     * Use this in an OpMode's init method
     *
     * @param hardwareMap                   The hardware map is used to get the context, which is needed to initialize the camera
     * @param encoderMeasurementCovariance  The amount of weight odometry is given in the position estimation. 1 is least, 0 is most.
     * @param offsetX                       The x offset of the camera in reference to the robot's center. Give inches
     * @param offsetY                       The y offset of the camera in reference to the robot's center. Give inches
     * @param offsetAng                     The angular offset of the camera in reference to the robot's front. Give degrees
     */
    public T265(HardwareMap hardwareMap, double encoderMeasurementCovariance, double offsetX, double offsetY, double offsetAng) {
        //This converts the inputs from inches and degrees to usable meters and radians and accounts for wacky camera positioning.
        double offsetXMeters = DistanceUnit.mPerInch * offsetY; //Not sure whether or not to negate the offsets or swap them.
        double offsetYMeters = DistanceUnit.mPerInch * -offsetX;
        offsetAng = Math.toRadians(offsetAng);

        //Initialize the starting position (0, 0, 0)
        //Initialize the camera's offset from the center of the robot
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());
        Transform2d cameraToRobot = new Transform2d(new Translation2d(offsetXMeters, offsetYMeters), new Rotation2d(offsetAng));

        //Initialize the actual camera object. If there was an error, it failed
        //NOTE: If this was an error, make sure to remove 'arm64-v8a' from each of two places in the project's build.common.gradle
        try {
            slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        }
        catch (Exception e) {
                failed = true;
        }

        //If there was no error, set the starting position of the robot to the previously declared (0, 0, 0)
        if (!failed) {
            slamra.setPose(startingPose);
        }
    }

    /**
     * Resets the position of the robot (not the offset of the camera).
     * Use this anywhere in an OpMode
     *
     * @param x     The x position, in inches, of the robot
     * @param y     The y position, in inches, of the robot
     * @param ang   The heading, in degrees, of the robot
     */
    public void setPose(double x, double y, double ang) {
        //Convert the inputs from inches and degrees to usable meters and radians
        double xMeters = DistanceUnit.mPerInch * y;
        double yMeters = DistanceUnit.mPerInch * -x;
        ang = Math.toRadians(ang);

        //Create and pass the 'starting position' to the camera
        Pose2d startingPose = new Pose2d(xMeters, yMeters, new Rotation2d(ang));
        slamra.setPose(startingPose);
    }

    /**
     * Starts the camera. Use this in an OpMode's start method
     */
    public void start() {
        //If the camera didn't fail, try to start it.
        //If starting it caused an error, it failed. NOTE: If this is an error, make sure to use version 2.0.1+ of the ftc265 library
        if (!failed) {
            try {
                slamra.start();
            }
            catch (Exception e) {
                failed = true;
            }
        }
    }

    /**
     * Stops the camera. Use this in an OpMode's stop method
     */
    public void stop() {
        //NOTE: I'm not sure if this will cause an error if the camera failed
        slamra.stop();
    }

    /**
     * Gets the most recent update from the camera and saves its variables to the class.
     * The camera updates in a separate (inaccessible) thread, which is constantly running.
     * You can use this anywhere, but it's recommended to use this in an OpMode's loop method
     */
    public void update() {
        //If the camera didn't fail to start, get the most recent update
        if (!failed) {
            T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();

            //Try to use the update to get the position, heading, velocity, and confidence
            //If there was an error, the update is null. NOTE: This almost never happens, and doesn't cause an issue to the estimation
            try {
                translation = update.pose.getTranslation();
                rotation = update.pose.getRotation();
                velocity = update.velocity;
                confidence = update.confidence;

                updateIsNull = false;
            }
            catch (Exception e) {
                updateIsNull = true;
            }
        }
    }

    /**
     * Send odometry data to the T265 camera for more accurate estimation
     * @param velX  The x velocity of the robot in inches / sec
     * @param velY  The y velocity of the robot in inches / sec
     */
    public void sendOdometry(double velX, double velY) {
        //Convert inches / sec to meters / sec and account for wacky camera positioning
        double velXMPerS = velY * DistanceUnit.mPerInch;
        double velYMPerS = -velX * DistanceUnit.mPerInch;
        slamra.sendOdometry(velXMPerS, velYMPerS);
    }

    /**
     * Gets the translation of the robot
     * NOTE: X and Y are swapped, and they are each backward
     * @return A Translation2d object, with x and y positions in meters
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Gets the rotation of the robot
     * @return A Rotation2d object, with heading in radians and degrees (-180 to 180)
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Gets the velocity of the robot
     * @return A ChassisSpeeds object, with vx and vy in meters / sec and vang in radians / sec
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * Gets the confidence of the estimated position
     * @return A PoseConfidence enum object
     */
    public T265Camera.PoseConfidence getConfidence() {
        return confidence;
    }

    /**
     * Gets the failed variable
     * @return A boolean denoting whether or not the camera failed to start properly
     */
    public boolean isFailed() {
        return failed;
    }

    /**
     * gets the updateIsNull variable
     * @return A boolean denoting whether or not the most recent update was null
     */
    public boolean isUpdateIsNull() {
        return updateIsNull;
    }

    /**
     * Gets the x position of the robot
     * @return A double of the x position, in inches
     */
    public double getX() {
        //Convert from meters to inches and account for weird camera position
        return -translation.getY() / DistanceUnit.mPerInch;
    }

    /**
     * Gets the y position of the robot
     * @return A double of the y position, in inches
     */
    public double getY() {
        //Convert from meters to inches and account for weird camera position
        return translation.getX() / DistanceUnit.mPerInch;
    }

    /**
     * Gets the heading of the robot
     * @return A double of the heading, in degrees (0-360)
     */
    public double getAng() {
        double angle = rotation.getDegrees();

        //Angle is given in -180 to 180 format. This converts it to 0-360
        if (angle <= 0) {
            return Math.abs(angle);
        }
        else {
            return 360 - angle;
        }
    }

    /**
     * Gets the x velocity of the robot
     * @return A double of the x velocity, in inches / sec
     */
    public double getVelX() {
        //Convert from meters / sec to inches / sec
        return -velocity.vyMetersPerSecond / DistanceUnit.mPerInch;
    }

    /**
     * Gets the y velocity of the robot
     * @return A double of the y velocity, in inches / sec
     */
    public double getVelY() {
        //Convert from meters / sec to inches / sec
        return velocity.vxMetersPerSecond / DistanceUnit.mPerInch;
    }

    /**
     * Gets the angular velocity of the robot
     * @return A double of the angular velocity, in degrees / sec
     */
    public double getVelAng() {
        //Convert from radians / sec to degrees / sec
        double angle = Math.toDegrees(velocity.omegaRadiansPerSecond);

        //Angle is given in -180 to 180 format. This converts it to 0-360
        if (angle <= 0) {
            return Math.abs(angle);
        }
        else {
            return 360 - angle;
        }
    }
}
