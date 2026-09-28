package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public abstract class PoseGetter {
    public abstract @Nullable Pose getPose();

    public abstract @Nullable Vector getVel();
}
