package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.ArrayList;

public class BulkRead {

    LynxModule controlHub;

    public DcMotor x, r, y;
    public int portX1, portR, portY;

    public BulkRead(LynxModule controlHub, DcMotor x, DcMotor r, DcMotor y) {

        this.controlHub = controlHub;

        this.x = x;
        this.r = r;
        this.y = y;

        portX1 = this.x.getPortNumber();
        portR = this.r.getPortNumber();
        portY = this.y.getPortNumber();


    }



    public synchronized double [] getMotors() {

        try
        {
            LynxModule.BulkData bData = controlHub.getBulkData();
            double posOne = (x.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorCurrentPosition(portX1) : bData.getMotorCurrentPosition(portX1);
            double posTwo = (r.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorCurrentPosition(portR) : bData.getMotorCurrentPosition(portR);
            double posThree = (y.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorCurrentPosition(portY) : bData.getMotorCurrentPosition(portY);
            return new double[]{posOne, posTwo, posThree};
        }

        catch (Exception e)
        {
            return new double[]{943028, 932432908};
        }






    }

    public synchronized double[] getMVelocity() {

        try
        {
            LynxModule.BulkData bData = controlHub.getBulkData();
            double posOne = (x.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorVelocity(portX1) : bData.getMotorVelocity(portX1);
            double posTwo = (r.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorVelocity(portR) : bData.getMotorVelocity(portR);
            double posThree = (y.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorVelocity(portY) : bData.getMotorVelocity(portY);
            return new double[]{posOne, posTwo, posThree};
        }

        catch (Exception e)
        {
            return new double[]{943028, 932432908};
        }

    }

    //TODO: Change this to the above method of response.
    public synchronized double[] getGenericMotors(DcMotor d1, DcMotor d2, DcMotor d3, DcMotor d4) {

        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(controlHub);
        LynxGetBulkInputDataResponse lynxResponse;
        try
        {
            lynxResponse = command.sendReceive();
        }

        catch (Exception e)
        {
            return new double[]{32498329, 239493};
        }

        double posOne = (d1.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d1.getPortNumber()) : lynxResponse.getEncoder(d1.getPortNumber());
        double posTwo = (d2.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d2.getPortNumber()) : lynxResponse.getEncoder(d2.getPortNumber());
        double posThree = (d3.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d3.getPortNumber()) : lynxResponse.getEncoder(d3.getPortNumber());
        double posFour = (d4.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(d4.getPortNumber()) : lynxResponse.getEncoder(d4.getPortNumber());

        return new double[]{posOne, posTwo, posThree, posFour};

    }

}
