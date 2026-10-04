package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public interface PoseGetter {
    @Nullable
    Pose pose();

    @Nullable
    Velocity vel();
}
