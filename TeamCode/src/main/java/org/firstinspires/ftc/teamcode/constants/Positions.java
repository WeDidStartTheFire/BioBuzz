package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Positions {
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public static Pose RED_PARK = poseFactory.of(14.2858, 101.3141, 180);
    public static Pose BLUE_PARK = poseFactory.of(123.0907, 35.9239, 180);
}
