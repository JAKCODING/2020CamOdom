package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.openftc.revextensions2.RevBulkData;

public class BulkReadTest {

    LynxGetBulkInputDataResponse lynxResponse;
    LynxModule controlHub;

    public BulkReadTest(LynxGetBulkInputDataResponse lynxResponse, LynxModule controlHub) {

        this.lynxResponse = lynxResponse;
        this.controlHub = controlHub;

    }

    public String bulkResponse() {

        return lynxResponse.toString();

    }

    public void letsMessAroundWithResponse() {

        lynxResponse.

    }
}
