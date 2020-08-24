package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tank {

    //DC Motors and velocity variables
    DcMotor fLeft, fRight, bLeft, bRight;
    double v1, v2, v3, v4; //In same order as motors

    //Variables for inputs
    double leftY, rightY, leftT, rightT;

    //Power limit
    double lim;

    //Telemetry object
    Telemetry telemetry;

    public Tank(HardwareMap hardwareMap, Telemetry telemetry, String fl, String fr, String bl, String br) {
        //Hardware mapping the motors
        fLeft = hardwareMap.dcMotor.get(fl);
        fRight = hardwareMap.dcMotor.get(fr);
        bLeft = hardwareMap.dcMotor.get(bl);
        bRight = hardwareMap.dcMotor.get(br);

        //Reversing left motors
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        //Initialize variables
        lim = 0.8;
        v1 = 0;
        v2 = 0;
        v3 = 0;
        v4 = 0;
    }

    public void drive(Gamepad gamepad1) {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        leftT = gamepad1.left_trigger;
        rightT = gamepad1.right_trigger;

        if (rightT > 0.1) {
            v1 = -rightT;
            v2 = rightT;
            v3 = rightT;
            v4 = -rightT;
        }
        else if (leftT > 0.1) {
            v1 = leftT;
            v2 = -leftT;
            v3 = -leftT;
            v4 = leftT;
        }
        else {
            v1 = leftY;
            v2 = rightY;
            v3 = leftY;
            v4 = rightY;
        }

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1/lim);
            v2 /= max * (1/lim);
            v3 /= max * (1/lim);
            v4 /= max * (1/lim);
        }

        //Telemetry for the motor velocities
        telemetry.addData("fLeft: ", v1);
        telemetry.addData("fRight: ", v2);
        telemetry.addData("bLeft: ", v3);
        telemetry.addData("bRight: ", v4);

        //Setting each velocity to its respective motor
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
