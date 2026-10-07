package org.firstinspires.ftc.teamcode.autos.paths;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;


public class Paths_8Pt_Up extends AutoPaths {

    private Pose start, path1, path1Control1;
    private static final Color pathsColor = Color.BLUE;

    public Paths_8Pt_Up(Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(80.25, 132.5, 90);
        path1 = poseFactory.of(126.5, 51, 270);
        path1Control1 = poseFactory.of(117, 107, 0);
    }

    @Override
    public Pose start() {
        return start;
    }

    public Path path1() {
        return curve(start, path1Control1, path1).constant(path1);
    }
}
