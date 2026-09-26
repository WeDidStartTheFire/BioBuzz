package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Artifact {
    GREEN,
    PURPLE,
    EMPTY,
    UNKNOWN;

    @NonNull
    @Override
    public String toString() {
        switch (this) {
            case GREEN:
                return "🟢Green🟢";
            case PURPLE:
                return "🟣Purple🟣";
            case EMPTY:
                return "◌Empty◌";
            case UNKNOWN:
                return "?UNKNOWN?";
            default:
                return super.toString();
        }
    }
}
