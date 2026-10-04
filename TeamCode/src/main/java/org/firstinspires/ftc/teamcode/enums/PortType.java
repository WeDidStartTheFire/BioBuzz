package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum PortType {
    SERVO, MOTOR, DIGITAL, I2C, USB;

    @NonNull
    public String toString() {
        switch (this) {
            case SERVO:
                return "Servo Port";
            case MOTOR:
                return "Motor Port";
            case DIGITAL:
                return "Digital Port";
            case I2C:
                return "I2C Port";
            case USB:
                return "USB Port";
            default:
                return "Unknown Port";
        }
    }
}
