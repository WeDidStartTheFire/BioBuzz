package org.firstinspires.ftc.teamcode.autos.paths;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;


public class Paths_8Pt_Up extends AutoPaths {

    private static final Color pathsColor = Color.BLUE;

    public Paths_8Pt_Up(Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
    }

    @Override
    public Pose start() {
        return path1().get(0);
    }

    public Path path1() {
        return curve(
                poseFactory.of(81.400, 133.750, 0.0),
                poseFactory.of(120.166, 91.695, 0.0),
                poseFactory.of(114.021, 104.478, 0.0),
                poseFactory.of(126.500, 51.000, 0.0)
        ).linear(Math.toRadians(-90), Math.toRadians(180));
    }
}
