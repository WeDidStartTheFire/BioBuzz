package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Color {
    RED, BLUE;

    @NonNull
    @Override
    public String toString() {
        return this == RED ? "🟥Red🟥" : "🟦Blue🟦";
    }
}
