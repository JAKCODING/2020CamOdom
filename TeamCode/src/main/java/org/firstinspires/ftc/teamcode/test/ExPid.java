package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="ExPID")
public class ExPid extends OpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double P=6.0;
    public static double I=3.0;
    public static double D=-3.0;

    //With large compliant wheel attached, 7 worked for every rpm
    //1560 was the max possible rpm when using a wheel
    public static double F=7.0;

    public int step = 0;

    DcMotorEx motor;

    ElapsedTime timer;

    public static double rpm=1400;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();
    }

    public void loop(){
        motor.setVelocityPIDFCoefficients(P, I, D, F);

        switch(step) {
            case 0:
                motor.setVelocity(rpm*(103.6/60));
                if (motor.getVelocity()*(60/103.6) >= rpm-5 && motor.getVelocity()*(60/103.6) <= rpm+5) {
                    timer.reset();
                    step++;
                }
                break;
            case 1:
                if (timer.seconds() >= 5) {
                    step++;
                }
                break;
            case 2:
                motor.setVelocity(0*(103.6/60));
                if (motor.getVelocity()*(60/103.6) >= 0-5 && motor.getVelocity()*(60/103.6) <= 0+5) {
                    timer.reset();
                    step++;
                }
                break;
            case 3:
                if (timer.seconds() >= 5) {
                    step=0;
                }
                break;
        }

        telemetry.addData("Velocity", motor.getVelocity()*(60/103.6));
    }

    public void stop(){
        motor.setPower(0);
    }

    /**
     * Running a motor without a wheel. F is based off of RPM.
     * @param rpm
     * @return
     */
    public double getFNoWheel(double rpm) {
        return Math.pow((rpm-1600)/-210, 2)+7;
    }
}
