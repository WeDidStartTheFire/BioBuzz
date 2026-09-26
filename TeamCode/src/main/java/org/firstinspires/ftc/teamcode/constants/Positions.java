package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class Positions {
    public static Pose3D RED_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 139, 139, 44, 0),
        new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    public static Pose3D BLUE_GOAL_POSE = new Pose3D(new Position(DistanceUnit.INCH, 5, 139, 44, 0),
        new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    public static Pose RED_HUMAN_PLAYER_POSE = new Pose(0, 9);
    public static Pose BLUE_HUMAN_PLAYER_POSE = new Pose(144, 9);

    public static Pose RED_BASE_ZONE = new Pose(38, 33, toRadians(90));
    public static Pose BLUE_BASE_ZONE = new Pose(106, 33, toRadians(90));
    public static Pose RED_FAR_LAUNCH = new Pose(84, 20, toRadians(90));
    public static Pose BLUE_FAR_LAUNCH = new Pose(60, 20, toRadians(90));
    public static Pose RED_HUMAN_PLAYER = new Pose(20, 14, toRadians(90));
    public static Pose BLUE_HUMAN_PLAYER = new Pose(124, 14, toRadians(90));
}
