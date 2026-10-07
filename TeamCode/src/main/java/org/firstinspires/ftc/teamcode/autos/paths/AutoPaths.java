package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.enums.Color;

public abstract class AutoPaths {

    protected final PoseFactory poseFactory;
    private static final Pose CENTER = new Pose(70.75, 70.75);

    public AutoPaths(Color pathsColor, Color autoColor) {
        PoseFactory tmp = PoseFactory.degrees();
        poseFactory = pathsColor == autoColor ? tmp : tmp.rotateAround(CENTER, 180);
        buildPoints();
    }

    public abstract Pose start();

    protected abstract void buildPoints();

    public static <T extends AutoPaths> T get(Class<T> autoPathsClass, Color autoColor) {
        try {
            return autoPathsClass.getConstructor(Color.class).newInstance(autoColor);
        } catch (Exception e) {
            throw new RuntimeException("autoPathsClass " + autoPathsClass + " needs to have a" +
                    " constructor following format AutoPaths(Color). Error message: " + e);
        }
    }
}
