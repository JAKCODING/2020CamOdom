package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

@TeleOp(name = "SimpleLocalizerTest")
public class SimpleLocalizerTest extends OpMode {

    EnforcersLocalizer localizer;

    DcMotor fLeft, fRight, bLeft, bRight;

    @Override
    public void init() {
        //localizer = new EnforcersLocalizer(hardwareMap);

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

        //Output telemetry of position and raw encoder values
        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addData("Ang: ", Math.toDegrees(pose.getHeading()));
        telemetry.addLine();
        telemetry.addData("Left Enc: ", wheelPositions.get(0));
        telemetry.addData("Right Enc: ", wheelPositions.get(1));
        telemetry.addData("Front Enc: ", wheelPositions.get(2));

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