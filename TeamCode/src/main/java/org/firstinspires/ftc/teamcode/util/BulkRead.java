package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.RevBulkData;

public class BulkRead {

    LynxGetBulkInputDataResponse lynxResponse;
    LynxModule controlHub;

    DcMotor x1, x2, y;
    int portX1, portX2, portY;

    public BulkRead(LynxGetBulkInputDataResponse lynxResponse, LynxModule controlHub, DcMotor x1, DcMotor x2, DcMotor y) {

        this.lynxResponse = lynxResponse;
        this.controlHub = controlHub;

        this.x1 = x1;
        this.x2 = x2;
        this.y = y;

        portX1 = this.x1.getPortNumber();
        portX2 = this.x2.getPortNumber();
        portY = this.y.getPortNumber();

    }

    public String bulkResponse() {

        return lynxResponse.toString();

    }

    public double[] getMotors() {

        double posOne = (x1.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portX1) : lynxResponse.getEncoder(portX1);
        double posTwo = (x2.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portX2) : lynxResponse.getEncoder(portX2);
        double posThree = (y.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portY) : lynxResponse.getEncoder(portY);

        return new double[]{posOne, posTwo, posThree};

    }

    public double[] getGenericMotors(DcMotor d1, DcMotor d2, DcMotor d3, DcMotor d4) {

        double posOne = (d1.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d1.getPortNumber()) : lynxResponse.getEncoder(d1.getPortNumber());
        double posTwo = (d2.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d2.getPortNumber()) : lynxResponse.getEncoder(d2.getPortNumber());
        double posThree = (d3.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d3.getPortNumber()) : lynxResponse.getEncoder(d3.getPortNumber());
        double posFour = (d4.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d4.getPortNumber()) : lynxResponse.getEncoder(d4.getPortNumber());

        return new double[]{posOne, posTwo, posThree, posFour};

    }

}
