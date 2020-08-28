package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Odometer Test")
public class OdomTest extends OpMode {

    DcMotorEx leftEnc, rightEnc, frontEnc;

    public void init() {
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
