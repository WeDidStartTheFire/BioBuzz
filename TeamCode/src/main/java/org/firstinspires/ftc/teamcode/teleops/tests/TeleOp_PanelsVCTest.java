package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Panels Virtual Controller Test", group = "Test")
public class TeleOp_PanelsVCTest extends OpMode {

    public TelemetryUtils tm;
    public Robot robot;
    public GamepadManager gamepadManager;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        tm = robot.drivetrain.tm;
        gamepadManager = PanelsGamepad.INSTANCE.getFirstManager();
        tm.print("Panels Virtual Controller Test Initialized");
        tm.update();
    }

    @Override
    public void loop() {
        Gamepad gamepad = gamepadManager.asCombinedFTCGamepad(gamepad1);
        tm.print("A Button: ", gamepad.a);
        tm.print("B Button: ", gamepad.b);
        tm.print("X Button: ", gamepad.x);
        tm.print("Y Button: ", gamepad.y);
        tm.updateOnlyPanels(3);
    }
}
