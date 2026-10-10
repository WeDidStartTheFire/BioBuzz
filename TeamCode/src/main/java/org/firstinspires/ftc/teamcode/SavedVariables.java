package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import dev.frozenmilk.sinister.loading.Pinned;

@Configurable
@Pinned
public class SavedVariables {
    @Nullable
    public static Pose savedPose;
}
