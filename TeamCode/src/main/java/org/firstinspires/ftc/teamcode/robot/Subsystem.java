package org.firstinspires.ftc.teamcode.robot;

public interface Subsystem {
    void readSensors();

    void think();
    void update();
    void stop();
}
