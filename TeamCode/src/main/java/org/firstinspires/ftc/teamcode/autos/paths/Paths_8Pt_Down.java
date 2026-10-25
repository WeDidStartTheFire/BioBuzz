package org.firstinspires.ftc.teamcode.autos.paths;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;


public class Paths_8Pt_Down extends AutoPaths {

    private static final Color pathsColor = Color.BLUE;

    public Paths_8Pt_Down(Color autoColor) {
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
                poseFactory.of(81.400, 10.250, 0.0),
                poseFactory.of(75.325, 23.893, 0.0),
                poseFactory.of(126.370, 21.000, 0.0)
        ).constant(Math.toRadians(90));
    }
}
