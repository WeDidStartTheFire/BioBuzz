package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.Utils.loadOdometryPosition;
import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Test Bus Speed", group = "Test")
@Disabled
@Configurable
public class TeleOp_TestBusSpeed extends OpMode {
    private Robot robot;
    private TelemetryUtils tm;
    private double totalColorTime = 0;
    private double totalOTOSTime = 0;
    private int totalColorCalls = 0;
    private int totalOTOSCalls = 0;
    private boolean fastMode = false;
    @SuppressWarnings("CanBeFinal")
    public static int ARTIFICIAL_WAIT = 50;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        robot.drivetrain.setPose(loadOdometryPosition());
        tm = robot.drivetrain.tm;
        tm.setEnvironment(RobotEnvironment.DEBUG);
        if (!robot.drivetrain.isPoseValid())
            tm.warn(TelemetryUtils.ErrorLevel.LOW, "Robot Centric driving will be used until the position is reset");
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "🟦Blue🟦", INFO);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            fastMode = true;
            totalColorCalls = 0;
            totalOTOSCalls = 0;
            totalColorTime = 0;
            totalOTOSTime = 0;
        }
        if (gamepad1.bWasPressed()) {
            robot.colorSensor.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
            robot.drivetrain.setOtosBusSpeed(LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
            fastMode = false;
            totalColorCalls = 0;
            totalOTOSCalls = 0;
            totalColorTime = 0;
            totalOTOSTime = 0;
        }
        double t0 = getRuntime();
        robot.colorSensor.getRGB(true);
        double t1 = getRuntime();
        totalColorTime += t1 - t0;
        totalColorCalls++;
        tm.print("Bus Speed", fastMode ? "Fast" : "Standard", VERBOSE);
        tm.print("Color Sensor Time (ms)", (t1 - t0) * 1000, VERBOSE);
        tm.print("Color Sensor Average Time (ms)", totalColorTime / totalColorCalls * 1000, VERBOSE);
        t0 = getRuntime();
        if (robot.drivetrain.otos != null) robot.drivetrain.follower.update();
        t1 = getRuntime();
        totalOTOSTime += t1 - t0;
        totalOTOSCalls++;
        tm.print("OTOS Time (ms)", (t1 - t0) * 1000, VERBOSE);
        tm.print("OTOS Average Time (ms)", totalOTOSTime / totalOTOSCalls * 1000, VERBOSE);
        tm.updateOnlyPanels();
        try {
            sleep(ARTIFICIAL_WAIT);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
