package org.firstinspires.ftc.teamcode.pedro.localizers;

import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.DEBUG;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.HashMap;
import java.util.Map;

public abstract class LoggingLocalizer implements Localizer {
    @Nullable
    protected TelemetryUtils tm;
    protected boolean logging = false;
    protected String[] columns;
    @NonNull
    private final Map<Object, Object> log = new HashMap<>();

    public void enableTelemetry(TelemetryUtils tm) {
        this.tm = tm;
    }

    public void initializeLogging() {
        this.logging = true;
    }

    public String getHeader() {
        StringBuilder header = new StringBuilder();
        for (String title : columns) header.append(title).append(',');
        return header.toString();
    }

    public String getLog() {
        StringBuilder row = new StringBuilder();
        for (String title : columns) row.append(log.getOrDefault(title, "")).append(',');
        return row.toString();
    }

    protected void addLog(@NonNull Object caption, @NonNull Object content) {
        if (tm != null) tm.print(caption, content, DEBUG);
        if (logging) {
            if (content instanceof Pose) {
                double x = ((Pose) content).x();
                double y = ((Pose) content).y();
                double h = ((Pose) content).heading();
                log.put(caption + " x", x);
                log.put(caption + " y", y);
                log.put(caption + " h", h);
            } else if (content instanceof Velocity) {
                double x = ((Velocity) content).vx;
                double y = ((Velocity) content).vy;
                double h = ((Velocity) content).omega;
                log.put(caption + " x", x);
                log.put(caption + " y", y);
                log.put(caption + " h", h);
            } else log.put(caption, content);
        }
    }

    protected void resetLog() {
        log.clear();
    }
}
