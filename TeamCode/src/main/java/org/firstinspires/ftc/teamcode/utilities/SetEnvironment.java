package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;

@Utility(name = "Set Competition Environment (for Telemetry)")
public class SetEnvironment extends OpMode {
    TelemetryUtils tm;

    @Override
    public void init() {
        tm = new TelemetryUtils(telemetry);
        tm.setGlobalEnvironment(RobotEnvironment.COMPETITION);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed())
            tm.setGlobalEnvironment(RobotEnvironment.PRACTICE);
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed())
            tm.setGlobalEnvironment(RobotEnvironment.COMPETITION);
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed())
            tm.setGlobalEnvironment(RobotEnvironment.DEBUG);

        tm.print("Current Environment", TelemetryUtils.globalEnvironment, INFO);
        tm.print("====================", INFO);
        tm.print("The environment determines the frequency and type/verbosity of " +
            "telemetry to Panels and the Driver Hub.", INFO);
        tm.print("To change environment:", INFO);
        tm.print("Press A for PRACTICE", INFO);
        tm.print("Press B for COMPETITION", INFO);
        tm.print("Press X for DEBUG", INFO);
        tm.updateOnlyPanels();
    }

    @Override
    public void loop() {
        terminateOpModeNow();
    }
}
