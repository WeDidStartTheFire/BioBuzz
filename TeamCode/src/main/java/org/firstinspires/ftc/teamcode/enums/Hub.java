package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Hub {
    CONTROL("Control Hub", 173),
    EXPANSION("Expansion Hub", 2),
    UNKNOWN("Unknown Hub", 0);

    private final String name;
    private final int address;

    Hub(String name, int address) {
        this.name = name;
        this.address = address;
    }

    public int id() {
        return address;
    }

    @NonNull
    public String toString() {
        return name;
    }
}
