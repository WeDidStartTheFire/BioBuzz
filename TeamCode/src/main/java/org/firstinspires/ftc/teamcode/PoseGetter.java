package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public interface PoseGetter {
    @Nullable
    Pose getPose();

    @Nullable
    Vector getVel();
}
