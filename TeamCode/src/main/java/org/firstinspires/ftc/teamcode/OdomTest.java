package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "odometer test")
public class OdomTest extends OpMode {

    DcMotor enc;

    public void init() {
        enc = hardwareMap.dcMotor.get("e");
    }

    public void loop() {

        telemetry.addData("encoder output: ", enc.getCurrentPosition());

    }

    public void stop() {

    }

}
