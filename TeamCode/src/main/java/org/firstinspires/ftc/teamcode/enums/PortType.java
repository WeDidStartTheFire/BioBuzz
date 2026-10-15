package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum PortType {
    SERVO("Servo Port"),
    MOTOR("Motor Port"),
    DIGITAL("Digital Port"),
    I2C("I2C Port"),
    USB("USB Port"),
    RS485("RS485 Port");

    private final String name;

    PortType(String name) {
        this.name = name;
    }

    @NonNull
    public String toString() {
        return name;
    }
}
