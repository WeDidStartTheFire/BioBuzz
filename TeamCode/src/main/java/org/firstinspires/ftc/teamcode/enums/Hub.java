package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Hub {
    CONTROL("Control Hub"),
    EXPANSION("Expansion Hub"),
    UNKNOWN("Unknown Hub");

    private final String name;

    Hub(String name) {
        this.name = name;
    }

    @NonNull
    public String toString() {
        return name;
    }
}
