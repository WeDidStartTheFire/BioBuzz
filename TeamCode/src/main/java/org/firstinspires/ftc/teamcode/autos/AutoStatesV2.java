package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.Nullable;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.HashMap;
import java.util.Objects;

public class AutoStatesV2 {
    public String currentState = "IDLE";

    private final Robot robot;
    private final TelemetryUtils tm;

    HashMap<String, Runnable> stateMap = new HashMap<>();

    public AutoStatesV2(Robot robot, @Nullable HashMap<String, Runnable> customStateMap, TelemetryUtils tm) {
        this.tm = tm;
        if (customStateMap != null) stateMap.putAll(customStateMap);
        this.robot = robot;
    }

    private Runnable intake() {
        return () -> robot.intake.setBehavior(Intake.Behavior.INTAKE);
    }

    private Runnable shoot() {
        return robot.launcher::spin;
    }

    private Runnable idle() {
        return () -> {
            robot.launcher.stop();
            robot.intake.setBehavior(Intake.Behavior.OFF);
        };
    }

    /**
     * Inserts the default commands. This should always go in the init function of you
     * auto or teleop!
     **/
    public void insertDefaultStates() {
        stateMap.put("SHOOT", shoot());
        stateMap.put("INTAKE", intake());
        stateMap.put("IDLE", idle());
    }

    public Command setCurrentState(String state) {
        return Command.build()
                .setStart(() -> setCurrentState(state))
                .setDone(() -> Objects.equals(currentState, state));
    }

    public Command update() {
        return Command.build()
                .setExecute(() -> {
                    if (stateMap.containsKey(currentState)) {
                        if (stateMap.get(currentState) != null) {
                            stateMap.get(currentState).run();
                        }
                    }
                });
    }
}
