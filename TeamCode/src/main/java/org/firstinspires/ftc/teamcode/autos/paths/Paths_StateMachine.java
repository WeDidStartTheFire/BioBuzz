package org.firstinspires.ftc.teamcode.autos.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.enums.Color;

public class Paths_StateMachine extends AutoPaths {

    private static final Color pathsColor = Color.BLUE;
    private Pose start, path1, point2, point3, point3Control1, point4, point4Control1,
            point5, point5Control1, point6, point6Control1, point6Control2, point7,
            point7Control1, point8, point8Control1, point9, point9Control1;

    public Paths_StateMachine(Color pathsColor, Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(129.3609, 35, 90);
        path1 = poseFactory.of(71, 35, 180);
        point2 = poseFactory.of(71, 101, 90);
        point3 = poseFactory.of(28, 101, -125.6285);
        point3Control1 = poseFactory.of(50, 132, 0);
        point4 = poseFactory.of(47, 23, -39.3956);
        point4Control1 = poseFactory.of(4, 58, 0);
        point5 = poseFactory.of(118, 61, 125.0028);
        point5Control1 = poseFactory.of(136, 36, 0);
        point6 = poseFactory.of(118, 115, 105.957);
        point6Control1 = poseFactory.of(88, 74, 0);
        point6Control2 = poseFactory.of(127, 85, 0);
        point7 = poseFactory.of(79, 114, 130.9165);
        point7Control1 = poseFactory.of(97, 93, 0);
        point8 = poseFactory.of(71, 47, -88.0606);
        point8Control1 = poseFactory.of(70, 75, 0);
        point9 = poseFactory.of(126, 35, 10.8126);
        point9Control1 = poseFactory.of(69, 24, 0);
    }

    @Override
    public Pose start() {
        return start;
    }

    public Path path1() {
        return line(start, path1);
    }

    public Path point2() {
        return line(path1, point2);
    }

    public Path point3() {
        return curve(point2, point3Control1, point3);
    }

    public Path point4() {
        return curve(point3, point4Control1, point4);
    }

    public Path point5() {
        return curve(point4, point5Control1, point5);
    }

    public Path point6() {
        return curve(point5, point6Control1, point6Control2, point6);
    }

    public Path point7() {
        return curve(point6, point7Control1, point7);
    }

    public Path point8() {
        return curve(point7, point8Control1, point8);
    }

    public Path point9() {
        return curve(point8, point9Control1, point9);
    }
}
