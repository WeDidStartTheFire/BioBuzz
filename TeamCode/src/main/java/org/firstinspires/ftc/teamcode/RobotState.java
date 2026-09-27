package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotState {
    public static boolean validStartPose;
    @Nullable
    public static Pose savedPose;
}
