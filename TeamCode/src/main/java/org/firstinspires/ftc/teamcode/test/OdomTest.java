package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "Odometer Test")
public class OdomTest extends OpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotorEx leftEnc, rightEnc, frontEnc;

    Encoder enc;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        leftEnc = hardwareMap.get(DcMotorEx.class, "bRight");
        rightEnc = hardwareMap.get(DcMotorEx.class, "bLeft");
        frontEnc = hardwareMap.get(DcMotorEx.class, "fRight");
    }

    public void loop() {
        telemetry.addData("Left Enc: ", leftEnc.getCurrentPosition());
        telemetry.addData("Right Enc: ", rightEnc.getCurrentPosition());
        telemetry.addData("Front Enc: ", frontEnc.getCurrentPosition());
    }
}
