package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Hub {
    CONTROL, EXPANSION, UNKNOWN;

    @NonNull
    public String toString() {
        switch (this) {
            case CONTROL:
                return "Control Hub";
            case EXPANSION:
                return "Expansion Hub";
            default:
                return "Unknown Hub";
        }
    }
}
