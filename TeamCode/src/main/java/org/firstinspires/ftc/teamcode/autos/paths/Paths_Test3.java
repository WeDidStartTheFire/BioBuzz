package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.enums.Color;

public class Paths_Test3 extends AutoPaths {

    private static final Color pathsColor = Color.BLUE;
    private Pose start, path1, path1Segment1Target, path1Segment2Start, path1Segment2End,
            point2Start, point2, point3, point3Control1, point3Control2, point3Segment1Start,
            point3Segment1End, point3Segment2Target, point4, point4Control1, point4Control2,
            point4Control3, point4Segment1Target, point4Segment2Start, point4Segment2End, point5;

    public Paths_Test3(Color autoColor) {
        super(pathsColor, autoColor);
    }

    @Override
    protected void buildPoints() {
        start = poseFactory.of(132.125, 9.375, 90);
        path1 = poseFactory.of(32, 40, 0);
        path1Segment1Target = poseFactory.of(144, 144, 0);
        path1Segment2Start = poseFactory.of(32, 40, 62.5653);
        path1Segment2End = poseFactory.of(32, 40, 0);
        point2Start = poseFactory.of(32, 40, 0);
        point2 = poseFactory.of(32, 108, 60);
        point3 = poseFactory.of(108.5, 32.5, -163.325);
        point3Control1 = poseFactory.of(170, 175, 0);
        point3Control2 = poseFactory.of(96, 56, 0);
        point3Segment1Start = poseFactory.of(108.5, 32.5, 60);
        point3Segment1End = poseFactory.of(108.5, 32.5, -136);
        point3Segment2Target = poseFactory.of(0, 0, 0);
        point4 = poseFactory.of(58.2378, 13.1292, 0);
        point4Control1 = poseFactory.of(77.9609, 55.4343, 0);
        point4Control2 = poseFactory.of(133.9934, 4.0753, 0);
        point4Control3 = poseFactory.of(134.1824, 70.4505, 0);
        point4Segment1Target = poseFactory.of(0, 0, 0);
        point4Segment2Start = poseFactory.of(58.2378, 13.1292, -166);
        point4Segment2End = poseFactory.of(58.2378, 13.1292, 0);
        point5 = poseFactory.of(33, 33, 0);
    }

    public Pose start() {
        return start;
    }

    public Path path1() {
        return Paths.line(start, path1).heading(Interpolator.piecewise().until(0.5,
                Interpolator.facingPoint(path1Segment1Target)).until(1,
                Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).
                heading(Interpolator.piecewise().until(0.3567,
                                Interpolator.linear(point3Segment1Start, point3Segment1End))
                        .until(1, Interpolator.facingPoint(point3Segment2Target)));
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4Control3, point4)
                .heading(Interpolator.piecewise().until(0.6373,
                                Interpolator.facingPoint(point4Segment1Target))
                        .until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

}
