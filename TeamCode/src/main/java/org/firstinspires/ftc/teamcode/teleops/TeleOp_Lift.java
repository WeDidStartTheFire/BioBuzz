package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class TeleOp_Lift extends OpMode {

    private TelemetryUtils tm;
    private Robot robot;
    private Lift lift;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);
        tm = new TelemetryUtils(telemetry);
        lift = new Lift(hardwareMap, tm);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {lift.setLiftMotorPower(1);}
        else if (gamepad1.dpadDownWasPressed()) {lift.setLiftMotorPower(-1);}
        else {lift.setLiftMotorPower(0);}
    }
}
