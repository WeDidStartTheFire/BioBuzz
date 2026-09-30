package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Panels Virtual Controller Test", group = "Test")
public class TeleOp_PanelsVCTest extends OpMode {

    public TelemetryUtils tm;
    public Robot robot;
    public GamepadManager vgamepad1;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, Color.BLUE));
        tm = robot.drivetrain.tm;
        vgamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
        tm.print("Panels Virtual Controller Test Initialized", INFO);
        tm.update();
    }

    @Override
    public void loop() {
        vgamepad1.update$Gamepad_release(vgamepad1.getCurrentState$Gamepad_release());
        tm.print("A Button: ", vgamepad1.getCross(), INFO);
        tm.print("B Button: ", vgamepad1.getCircle(), INFO);
        tm.print("X Button: ", vgamepad1.getSquare(), INFO);
        tm.print("Y Button: ", vgamepad1.getTriangle(), INFO);
        tm.updateOnlyPanels();
    }
}
