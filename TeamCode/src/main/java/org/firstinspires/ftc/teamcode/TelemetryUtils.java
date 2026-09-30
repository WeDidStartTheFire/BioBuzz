package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug;
import static java.lang.Math.min;
import static java.lang.Math.round;
import static java.lang.Math.toDegrees;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.RobotEnvironment;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.ArrayList;


@Configurable
public class TelemetryUtils {
    private final Telemetry telemetry;
    private static final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ArrayList<LogEntry> log = new ArrayList<>();
    private long lastDraw;
    private final StringBuilder poseStringBuilder = new StringBuilder();
    /// Environment the robot is in. Is persistent between OpModes and resets to
    /// {@link RobotEnvironment#PRACTICE} on power cycle.
    public static RobotEnvironment globalEnvironment = RobotEnvironment.PRACTICE;
    /// Environment the robot is in. Is temporary for one OpMode and resets to
    /// {@link TelemetryUtils#globalEnvironment} when OpMode ends.
    public static RobotEnvironment environment;

    public TelemetryUtils(Telemetry telemetry) {
        this.telemetry = telemetry;
        environment = globalEnvironment;
        telemetry.setAutoClear(true);
        telemetry.setNumDecimalPlaces(0, 5);
        updateTransmissionInterval();
        Drawing.init();
    }

    /**
     * Sets the current robot environment <i><b>for this OpMode only</b></i> and updates the
     * transmission interval for both the Control Hub and Panels.
     * <p> Higher telemetry transmission intervals (lower transmission frequencies)
     * reduce lag.</p>
     * <ul>
     *     <li> {@link RobotEnvironment#DEBUG} has the lowest transmission intervals and highest
     *     verbosity
     *     <li> {@link RobotEnvironment#COMPETITION} has the highest transmission intervals and
     *     lowest verbosity
     *     <li> {@link RobotEnvironment#PRACTICE} is in between
     * </ul>
     *
     * @param environment {@link RobotEnvironment}
     * @see #setGlobalEnvironment(RobotEnvironment)
     * @see RobotEnvironment
     */
    public void setEnvironment(RobotEnvironment environment) {
        TelemetryUtils.environment = environment;
        updateTransmissionInterval();
    }

    /**
     * Sets the robot environment <i><b>for all OpModes until robot power off</b></i> and updates
     * the transmission interval for both the Control Hub and Panels.
     * <p> Higher telemetry transmission intervals (lower transmission frequencies)
     * reduce lag.</p>
     * <ul>
     *     <li> {@link RobotEnvironment#DEBUG} has the lowest transmission intervals and highest
     *     verbosity
     *     <li> {@link RobotEnvironment#COMPETITION} has the highest transmission intervals and
     *     lowest verbosity
     *     <li> {@link RobotEnvironment#PRACTICE} is in between
     * </ul>
     *
     * @param environment {@link RobotEnvironment}
     * @see #setEnvironment(RobotEnvironment)
     * @see RobotEnvironment
     */
    public void setGlobalEnvironment(RobotEnvironment environment) {
        TelemetryUtils.globalEnvironment = environment;
        updateTransmissionInterval();
    }


    /**
     * Updates the transmission interval for both the Control Hub and Panels according to the
     * current robot environment.
     * <p> Higher transmission intervals (lower transmission frequencies)
     * reduce lag. {@link RobotEnvironment#DEBUG} has the lowest transmission intervals, while
     * {@link RobotEnvironment#COMPETITION} has the highest, and {@link RobotEnvironment#PRACTICE}
     * is in between.</p>
     *
     * @see RobotEnvironment
     */
    public void updateTransmissionInterval() {
        telemetry.setMsTransmissionInterval(environment.getDriverHubTransmissionIntervalMs());
        telemetryM.setUpdateInterval(environment.getPanelsTransmissionIntervalMs());
    }

    public enum PrintLevel {
        DEBUG, VERBOSE, INFO
    }

    /**
     * Adds to telemetry your data in the format "caption : content" on both the Control Hub and
     * Panels.
     *
     * @param caption String
     * @param content Object
     * @param level   {@link PrintLevel} - {@link PrintLevel#DEBUG DEBUG} messages will only be
     *                displayed in the {@link RobotEnvironment#DEBUG DEBUG} environment.
     *                {@link PrintLevel#VERBOSE VERBOSE} will not be displayed in
     *                {@link RobotEnvironment#COMPETITION COMPETITION} environment.
     * @see #print(Object, PrintLevel)
     * @see #print(Pose, PrintLevel)
     * @see #warn(ErrorLevel, Object)
     * @see PrintLevel
     * @see RobotEnvironment
     */
    public void print(@NonNull Object caption, @NonNull Object content, PrintLevel level) {
        if (level == PrintLevel.DEBUG && environment != RobotEnvironment.DEBUG) return;
        if (level == PrintLevel.VERBOSE && environment == RobotEnvironment.COMPETITION) return;
        telemetry.addData(caption.toString(), content);
        if (environment != RobotEnvironment.COMPETITION)
            telemetryM.addData(caption.toString(), content);
    }

    /**
     * Adds the caption to telemetry on both the Control Hub and Panels.
     *
     * @param content Content to display in telemetry
     * @param level   {@link PrintLevel} - {@link PrintLevel#DEBUG DEBUG} messages will only be
     *                displayed in the {@link RobotEnvironment#DEBUG DEBUG} environment.
     *                {@link PrintLevel#VERBOSE VERBOSE} will not be displayed in
     *                {@link RobotEnvironment#COMPETITION COMPETITION} environment.
     * @see #print(Object, Object, PrintLevel)
     * @see #print(Pose, PrintLevel)
     * @see #warn(ErrorLevel, Object)
     * @see PrintLevel
     * @see RobotEnvironment
     */
    public void print(@NonNull Object content, PrintLevel level) {
        if (level == PrintLevel.DEBUG && environment != RobotEnvironment.DEBUG) return;
        if (level == PrintLevel.VERBOSE && environment == RobotEnvironment.COMPETITION) return;
        telemetry.addLine(content.toString());
        if (environment != RobotEnvironment.COMPETITION) telemetryM.addLine(content.toString());
    }

    /**
     * Adds the pose to telemtry in the form of "Pose : (x, y, z)" rounded to two decimal places on
     * both the Control Hub and Panels
     *
     * @param pose  The pose to add to telemetry
     * @param level {@link PrintLevel} - {@link PrintLevel#DEBUG DEBUG} messages will only be
     *              displayed in the {@link RobotEnvironment#DEBUG DEBUG} environment.
     *              {@link PrintLevel#VERBOSE VERBOSE} will not be displayed in
     *              {@link RobotEnvironment#COMPETITION COMPETITION} environment.
     * @see #print(Object, Object, PrintLevel)
     * @see #print(Object, PrintLevel)
     * @see #warn(ErrorLevel, Object)
     * @see PrintLevel
     * @see RobotEnvironment
     */
    public void print(@NonNull Pose pose, PrintLevel level) {
        if (level == PrintLevel.DEBUG && environment != RobotEnvironment.DEBUG) return;
        if (level == PrintLevel.VERBOSE && environment == RobotEnvironment.COMPETITION) return;

        double x = round(pose.getX() * 100) / 100.0;
        double y = round(pose.getY() * 100) / 100.0;
        double h = round(toDegrees(pose.getHeading()) * 100) / 100.0;

        poseStringBuilder.setLength(0);
        poseStringBuilder.append('(').append(x).append(", ").append(y).append(", ").append(h).append(')');
        print("Pose", poseStringBuilder.toString(), level);
    }

    /**
     * Adds the caption and content to a log to be printed at the end of the program (only viewable
     * in Panels because the Control Hub updates telemtry after this is printed).
     *
     * @param caption The caption to log
     * @param content The content to log
     * @see #showLogs()
     * @see #showLogs(int)
     */
    public void log(String caption, Object content) {
        log.add(new LogEntry(caption, content));
    }

    /**
     * Prints and shows the logs in telemetry in the form of "caption : content" for the n most
     * recent items in the log from newest to oldest. Only shows logs in the
     * {@link RobotEnvironment#DEBUG DEBUG} environment.
     *
     * @param n Number of log entries to print
     * @see #showLogs()
     * @see #log(String, Object)
     * @see RobotEnvironment
     */
    public void showLogs(int n) {
        if (environment != RobotEnvironment.DEBUG) return;
        print("======= Logs =======", PrintLevel.DEBUG);
        n = min(log.size(), n);
        for (int i = log.size() - 1; i >= log.size() - n; i--) {
            LogEntry entry = log.get(i);
            print(entry.caption, entry.content, PrintLevel.DEBUG);
        }
        print("====================", PrintLevel.DEBUG);
    }

    /**
     * Prints and shows the logs in telemetry in the form of "caption : content" for each item (only
     * viewable if shown at program stop in Panels because the Control Hub updates telemtry after
     * this is printed). Only shows logs in the {@link RobotEnvironment#DEBUG DEBUG} environment.
     *
     * @see #log(String, Object)
     * @see #showLogs(int)
     * @see RobotEnvironment
     */
    public void showLogs() {
        if (environment != RobotEnvironment.DEBUG) return;
        print("======= Logs =======", PrintLevel.DEBUG);
        for (LogEntry entry : log) print(entry.caption, entry.content, PrintLevel.DEBUG);
        print("====================", PrintLevel.DEBUG);
    }

    /**
     * Updates telemetry on both the Control Hub and Panels.
     * <p>
     * <b>WARNING</b>: Avoid using this method every loop iteration as it can cause lag. Use
     * {@link #updateOnlyPanels(int numLogs)} or {@link #updateOnlyPanels()} instead when updating
     * telemetry in a loop. The default loop() and init_loop() methods of OpMode will automatically
     * update standard telemetry. Disregard this warning if you are not in either of those methods.
     * </p>
     *
     * @see #updateOnlyPanels(int numLogs)
     * @see #updateOnlyPanels()
     */
    public void update() {
        telemetry.update();
        if (environment != RobotEnvironment.COMPETITION) telemetryM.update();
        updateTransmissionInterval();
    }

    /**
     * Updates telemetry on Panels
     *
     * @param numLogs The number of logs to show
     * @see #updateOnlyPanels()
     */
    public void updateOnlyPanels(int numLogs) {
        if (environment == RobotEnvironment.COMPETITION) {
            updateTransmissionInterval();
            return;
        }
        if (numLogs > 0) showLogs(numLogs);
        telemetryM.update();
        updateTransmissionInterval();
    }

    /**
     * Updates telemetry on Panels. Shows the 3 most recent logs.
     *
     * @see #updateOnlyPanels(int numLogs)
     * @see #update()
     */
    public void updateOnlyPanels() {
        updateOnlyPanels(3);
    }


    /**
     * Draws the robot in panels
     *
     * @param follower The follower that has the pose history of the robot to draw
     */
    private void drawRobot(Follower follower) {
        drawDebug(follower);
    }

    /**
     * Draws the robot in panels if enough time has passed since the last draw
     *
     * @param follower Follower object to retrieve pose history from
     * @param ms       Minimum time between draws
     */
    public void drawRobot(Follower follower, int ms) {
        if (environment == RobotEnvironment.COMPETITION ||
            System.currentTimeMillis() - lastDraw < ms) return;
        lastDraw = System.currentTimeMillis();
        drawRobot(follower);
    }

    /**
     * Adds telemetry data from the last action
     *
     * @param message Message to be sent
     */
    public void addLastActionTelemetry(String message) {
        print("Last Action", message, PrintLevel.VERBOSE);
    }

    /**
     * Sends a warning message to Driver Station telemetry with a given serverity level.
     *
     * @param level   The severity level of the warning.
     * @param message The warning.
     * @see ErrorLevel
     */
    public void warn(ErrorLevel level, Object message) {
        print(level.toString(), message, PrintLevel.INFO);
    }

    /**
     * Enum representing the severity level of a warning. Can be {@link #LOW}, {@link #MEDIUM},
     * {@link #HIGH}, or {@link #CRITICAL}.
     *
     * @see #warn(ErrorLevel, Object)
     */
    public enum ErrorLevel {
        LOW("⚠️LOW WARNING⚠️"),
        MEDIUM("⚠️MEDIUM WARNING⚠️"),
        HIGH("🚨HIGH WARNING🚨"),
        CRITICAL("🚨CRITICAL WARNING🚨");

        private final String name;

        ErrorLevel(String name) {
            this.name = name;
        }

        @NonNull
        @Override
        public String toString() {
            return name;
        }
    }
}
