package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.paths.PathConstraints;

@Configurable
public class AutoConstants {
    public static PathConstraints slowIntakePathConstraints = new PathConstraints(
        0.3,
        500,
        0.5,
        0.4
    );
    public static double INTAKE_MOVE_MAX_SPEED = 0.4;
    public static double MAX_MOTIF_DETECT_WAIT = 1;
    public static double MAX_INTAKE_PATH_WAIT = 2.5;
    public static double INTAKE_AFTER_LAUNCH_WAIT = 1;
}
