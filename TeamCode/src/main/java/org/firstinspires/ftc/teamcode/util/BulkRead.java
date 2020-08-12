package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import java.util.ArrayList;

public class BulkRead {

    LynxModule controlHub;
    ArrayList<I2cDeviceSynch> i2cRead;


    DcMotor x1, r, y;
    public int portX1, portR, portY;

    public BulkRead(LynxModule controlHub, DcMotor x1, DcMotor r, DcMotor y, int[] i2cPorts) {

        this.controlHub = controlHub;

        this.x1 = x1;
        this.r = r;
        this.y = y;

        portX1 = this.x1.getPortNumber();
        portR = this.r.getPortNumber();
        portY = this.y.getPortNumber();

        /*for(int i = 0; i < i2cPorts.length; i++) {
            i2cRead.add(new BetterI2cDeviceSynchImplOnSimple(
                    new LynxI2cDeviceSynchV1(AppUtil.getDefContext(), controlHub, i2cPorts[i]), true));
        }*/


    }

    public void armI2C(int heartbeatMs) {

        for(int i = 0; i < i2cRead.size(); i++) {

            i2cRead.get(i).setHeartbeatInterval(heartbeatMs);

        }

    }


    public synchronized double [] getMotors() {

        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(controlHub);
        try
        {
            LynxGetBulkInputDataResponse lynxResponse = command.sendReceive();
            double posOne = (x1.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portX1) : lynxResponse.getEncoder(portX1);
            double posTwo = (r.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portR) : lynxResponse.getEncoder(portR);
            double posThree = (y.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getEncoder(portY) : lynxResponse.getEncoder(portY);
            return new double[]{posOne/*, posTwo*/, posThree};
        }

        catch (Exception e)
        {
            return new double[]{943028, 932432908};
        }






    }

    public synchronized double[] getMVelocity() {

        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(controlHub);
        try
        {
            LynxGetBulkInputDataResponse lynxResponse = command.sendReceive();
            double posOne = (x1.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getVelocity(portX1) : lynxResponse.getVelocity(portX1);
            double posTwo = (r.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getVelocity(portR) : lynxResponse.getVelocity(portR);
            double posThree = (y.getDirection() == DcMotorSimple.Direction.REVERSE) ? -lynxResponse.getVelocity(portY) : lynxResponse.getVelocity(portY);
            return new double[]{posOne/*, posTwo*/, posThree};
        }

        catch (Exception e)
        {
            return new double[]{943028, 932432908};
        }

    }

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

    /*public void i2cReadTest() {

        i2cRead.get(1).read()

    }*/

    public class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        public BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }


    public String handleException(Exception e) {

        return e.getStackTrace().toString();

    }

}
