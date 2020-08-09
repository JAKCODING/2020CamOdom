package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@Autonomous(name="bulkread")
public class BulkReadTest extends OpMode {

    LynxUsbDevice lUsb;
    ExpansionHubEx expX;
    LynxModule lMod;
    BulkRead bRead;
    LynxGetBulkInputDataResponse lResponse;
    DcMotor encX1, encY, encX2;

    public void init() {

        lUsb = (LynxUsbDevice) hardwareMap.get("Expansion Hub Portal 1");
        lMod = hardwareMap.get(LynxModule.class, "Control Hub");
        expX = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        lResponse = new LynxGetBulkInputDataResponse(lMod);

        encY = hardwareMap.dcMotor.get("encY");
        encX1 = hardwareMap.dcMotor.get("encX");


        bRead = new BulkRead(lMod, encX1, encX2, encY, new int[]{0});

    }


    public void loop() {
        lMod.setConstant(532342);

        double[] retVal = bRead.getMotors();
        String readVal = retVal[0] + " " + retVal[1];

        telemetry.addData("String return: ", readVal);
        telemetry.addLine(expX.getConnectionInfo());
        telemetry.addData("String return: ", expX.getBulkInputData().getMotorCurrentPosition(bRead.portX1));
        telemetry.addData("X port: ", bRead.portX1);
        telemetry.addData("Y port: ", bRead.portY);
    }
}
