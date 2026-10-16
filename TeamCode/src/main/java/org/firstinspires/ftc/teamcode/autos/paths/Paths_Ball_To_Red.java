package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;


public class Paths_Ball_To_Red extends AutoPaths {

    private Pose start, path1, path1Control1, path2Intake, path3Outake,
            path3OutakeControl1, point4, point5, point6Start, point6,
            point6Control1;
    private static final Color pathsColor = Color.BLUE;

    public Paths_Ball_To_Red(Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(132, 36, 90);
        path1 = poseFactory.of(71, 36, 180);
        path2Intake = poseFactory.of(71, 106, 90);
        path3Outake = poseFactory.of(124, 131, 92.7453);
        path3OutakeControl1 = poseFactory.of(126, 95, 0);
        point4 = poseFactory.of(123, 36, -90.6031);
        point5 = poseFactory.of(24, 36, 180);
        point6Start = poseFactory.of(24, 36, 180);
        point6 = poseFactory.of(11, 106, 0);
        point6Control1 = poseFactory.of(24, 82, 0);
    }

    @Override
    public Pose start() {
        return start;
    }

    public Path path1() {
        return Paths.line(start, path1).tangent();
    }

    public Path path2Intake() {
        return Paths.line(path1, path2Intake).tangent();
    }

    public Path path3Outake() {
        return Paths.curve(path2Intake, path3OutakeControl1, path3Outake).tangent();
    }

    public Path path4() {
        return Paths.line(path3Outake, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }
}
