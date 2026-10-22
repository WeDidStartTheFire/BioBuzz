package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.enums.Color;

public abstract class AutoPaths {

    protected final PoseFactory poseFactory;
    private static final Pose CENTER = new Pose(72, 72);

    public AutoPaths(Color pathsColor, Color autoColor) {
        PoseFactory tmp = PoseFactory.degrees();
        poseFactory = pathsColor == autoColor ? tmp : tmp.rotateAround(CENTER, 180);
        buildPoints();
    }

    /**
     * Returns the start pose of the path
     *
     * @return Start pose of the path
     */
    public abstract Pose start();

    /**
     * Builds the points of the path
     */
    protected abstract void buildPoints();

    /**
     * Returns an instance of the specified {@link AutoPaths} class
     *
     * @param autoPathsClass {@link AutoPaths} class
     * @param autoColor      {@link Color} of the autonomous
     * @param <T>            {@link AutoPaths} class
     * @return Instance of the specified {@link AutoPaths} class
     */
    public static <T extends AutoPaths> T get(Class<T> autoPathsClass, Color autoColor) {
        try {
            return autoPathsClass.getConstructor(Color.class).newInstance(autoColor);
        } catch (Exception e) {
            throw new RuntimeException("autoPathsClass " + autoPathsClass + " needs to have a" +
                    " constructor following format AutoPaths(Color). Error message: " + e);
        }
    }
}
