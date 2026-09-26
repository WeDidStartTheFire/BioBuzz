package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.enums.Artifact.UNKNOWN;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.enums.Motif;

@Configurable
public class RobotState {
    public static boolean auto = false;
    @Nullable
    public static Color color;
    public static boolean validStartPose;
    public static boolean robotCentric = false;
    @Nullable
    public static Pose pose;
    @Nullable
    public static Pose savedPose;
    @Nullable
    public static Vector vel;
    @NonNull
    public static Motif motif = Motif.UNKNOWN;
    @NonNull
    public static Artifact[] artifacts = {UNKNOWN, UNKNOWN, UNKNOWN};
    public static double launcherVelModifier = 0;
    public static boolean panelsResetTurret;
    public static boolean launcherIntaking = false;
    public static boolean normalIntaking = false;
}
