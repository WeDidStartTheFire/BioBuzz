package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.enums.Color;

public class MatchContext {
    public enum Mode {
        AUTO, TELEOP
    }

    private final Mode mode;
    private final Color alliance;

    public MatchContext(Mode mode, Color alliance) {
        this.mode = mode;
        this.alliance = alliance;
    }

    public Mode mode() {
        return mode;
    }

    public Color alliance() {
        return alliance;
    }

    public boolean isAuto() {
        return mode == Mode.AUTO;
    }
}
