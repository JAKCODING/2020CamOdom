package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

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

    DcMotor xFront, xBack, yBack;

    public ThreeWheelTracking(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, Y_OFFSET, Math.toRadians(90)),
                new Pose2d(0, -Y_OFFSET, Math.toRadians(90)),
                new Pose2d(X_OFFSET, -Y_OFFSET, Math.toRadians(0))
        ));

        xFront = hardwareMap.dcMotor.get("bLeft");
        xBack = hardwareMap.dcMotor.get("bRight");
        yBack = hardwareMap.dcMotor.get("fRight");
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(xFront.getCurrentPosition()),
                -encoderTicksToInches(xBack.getCurrentPosition()),
                -encoderTicksToInches(yBack.getCurrentPosition())
        );
    }

    private static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * (ticks / TICKS_PER_REV);
    }
}
