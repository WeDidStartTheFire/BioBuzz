package org.firstinspires.ftc.teamcode.enums;

/**
 * Different environments the robot can be in. It currently affects telemetry transmission intervals
 * and what messages are displayed.
 * <p> Higher telemetry transmission intervals (lower transmission frequencies) reduce lag.</p>
 * <ul>
 *     <li> {@link RobotEnvironment#DEBUG} has the lowest transmission intervals and highest
 *     verbosity
 *     <li> {@link RobotEnvironment#COMPETITION} has the highest transmission intervals and
 *     lowest verbosity
 *     <li> {@link RobotEnvironment#PRACTICE} is in between
 * </ul>
 *
 * @see org.firstinspires.ftc.teamcode.TelemetryUtils#setEnvironment(RobotEnvironment)
 * {@code TelemetryUtils.setEnvironment(RobotEnvironment environment)}
 * @see org.firstinspires.ftc.teamcode.TelemetryUtils#setGlobalEnvironment(RobotEnvironment)
 * {@code TelemetryUtils.setGlobalEnvironment(RobotEnvironment environment)}
 */
public enum RobotEnvironment {
    DEBUG(100, 50),
    PRACTICE(250, 250),
    COMPETITION(250, 1000);

    private final int driverHubTransmissionIntervalMs;
    private final int panelsTransmissionIntervalMs;

    RobotEnvironment(int driverHubTransmissionIntervalMs, int panelsTransmissionIntervalMs) {
        this.driverHubTransmissionIntervalMs = driverHubTransmissionIntervalMs;
        this.panelsTransmissionIntervalMs = panelsTransmissionIntervalMs;
    }

    public int getDriverHubTransmissionIntervalMs() {
        return driverHubTransmissionIntervalMs;
    }

    public int getPanelsTransmissionIntervalMs() {
        return panelsTransmissionIntervalMs;
    }
}
