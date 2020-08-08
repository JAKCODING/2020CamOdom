package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.openftc.revextensions2.ExpansionHubEx;

public class BulkReadTest extends OpMode {

    LynxUsbDevice lUsb;
    LynxModule lMod;
    BulkRead bRead;
    LynxGetBulkInputDataResponse lResponse;
    DcMotor encX1, encY, encX2;

    public void init() {

        lUsb = (LynxUsbDevice) hardwareMap.get("Expansion Hub Portal 1");
        lMod = new LynxModule(lUsb, 2, true);
        lResponse = new LynxGetBulkInputDataResponse(lMod);


        bRead = new BulkRead(lResponse, lMod, encX1, encX2, encY);

    }


    public void loop() {
        //Woo
    }
}
