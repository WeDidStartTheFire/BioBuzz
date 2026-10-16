package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Utility(name = "Panels Virtual Controller Test", description = "Tests Panels Virtual Controller")
public class PanelsVCTest extends OpMode {

    private TelemetryUtils tm;
    private GamepadManager gamepadManager;

    @Override
    public void init() {
        Robot robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        tm = robot.tm();
        gamepadManager = PanelsGamepad.INSTANCE.getFirstManager();
        tm.print("Panels Virtual Controller Test Initialized", INFO);
        tm.update();
    }

    @Override
    public void loop() {
        Gamepad gamepad = gamepadManager.asCombinedFTCGamepad(gamepad1);
        tm.print("A Button: ", gamepad.a, INFO);
        tm.print("B Button: ", gamepad.b, INFO);
        tm.print("X Button: ", gamepad.x, INFO);
        tm.print("Y Button: ", gamepad.y, INFO);
        tm.updateOnlyPanels();
    }
}
