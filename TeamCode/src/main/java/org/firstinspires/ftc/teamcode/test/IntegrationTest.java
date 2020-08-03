package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Pose2dConvert;

public class IntegrationTest extends OpMode {

    Pose2dConvert p2c = new Pose2dConvert();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    int counter = 0;

    public void init() {

    }

    public void loop() {


        switch(counter) {

            case 0:


        }

    }

}
