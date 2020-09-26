package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="Flywheel")
public class Flywheels extends OpMode {
    StandardTrackingWheelLocalizer myLocalizer;
    public void init() {
        // Insert whatever initialization your own code does

        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

    }
    public void loop(){

        myLocalizer.update();

        // Retrieve your pose
        Pose2d myPose = myLocalizer.getPoseEstimate();

        myLocalizer.getWheelVelocities();
        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());

        // Insert whatever teleop code you're using
    }
}
