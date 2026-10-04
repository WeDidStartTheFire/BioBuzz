package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public interface PoseGetter {
    @Nullable
    default Pose pose() {
        return null;
    }

    @Nullable
    default Velocity vel() {
        return null;
    }

    static PoseGetter from(PoseGetter poseGetter) {
        if (poseGetter == null) return new PoseGetter() {
        };
        return new PoseGetter() {
            @Nullable
            @Override
            public Pose pose() {
                return poseGetter.pose();
            }

            @Nullable
            @Override
            public Velocity vel() {
                return poseGetter.vel();
            }
        };
    }
}
