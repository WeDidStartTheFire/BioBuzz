package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class RobotState {
    public static boolean validStartPose;
    @Nullable
    public static Pose savedPose;
    @Nullable
    public static Pose pose;
    @Nullable
    public static Vector vel;
}
