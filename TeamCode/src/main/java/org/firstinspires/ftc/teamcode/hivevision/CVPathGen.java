package org.firstinspires.ftc.teamcode.hivevision;

import static org.firstinspires.ftc.teamcode.hivevision.CVConstants.RIGHT_OFFSET;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class CVPathGen {
    private Limelight3A ll;
    private Follower follower;
    private final boolean isRed;
    CVCostAlgo algo = new CVCostAlgo();

    Telemetry telemetry;

    public ArrayList<Pose> simBall = new ArrayList<>();
    public boolean simulation = false;
    public Pose simBotPose = new Pose(0, 0, 0);

    private static Matrix cameraAdjustments(Pose robotPose) {
        double heading = robotPose.heading();

        Matrix m = Matrix.rotationTransform(heading - PI / 2).times(CVConstants.createRotationX(-CVConstants.CAMERA_PITCH));
        m = m.times(CVConstants.xyzConverter);
        return m;
    }

    public CVPathGen(HardwareMap hardwareMap, Follower follower, boolean isRed, Telemetry telemetry) {
        this.follower = follower;
        this.isRed = isRed;
        this.telemetry = telemetry;

        ll = hardwareMap.get(Limelight3A.class, "limelight"); //change to your name
        ll.start();
        ll.pipelineSwitch(3); // change to your pipeline
    }

    public CVPathGen(boolean isRed) {
        this.isRed = isRed;
    }

    public ArrayList<Pose> getBalls(boolean sim) {
        if (sim) return simBall;
        ArrayList<double[]> relativeBalls = new ArrayList<>();
        ArrayList<Pose> absoluteBalls = new ArrayList<>();
        LLResult result = ll.getLatestResult();

        if (result != null && result.isValid()) {
            Pose po = follower.pose();
            for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                if (!"blue_nectar".equalsIgnoreCase(dr.getClassName())) { // Was yellow_pollen
                    continue;
                }
                relativeBalls.add(new double[]{Math.toRadians(dr.getTargetXDegrees()), Math.toRadians(dr.getTargetYDegrees())});
            }
            for (double[] ball : relativeBalls) {
                absoluteBalls.add(ballConverter(ball, po));
            }
        }
        drawAll(absoluteBalls);
        return absoluteBalls;
    }

    public Pose ballConverter(double[] ball, Pose curpose) {
        Matrix cameraRay = new Matrix(
                new double[][]{
                        {Math.tan(ball[0])},
                        {Math.tan(ball[1])},
                        {1.0}
                }
        );
        Matrix absoluteRay = cameraAdjustments(curpose).times(cameraRay);

        double dist = -CVConstants.CAMERA_HEIGHT / absoluteRay.get(2, 0);

        Matrix cam = new Matrix(
                new double[][]{
                        {curpose.x() + CVConstants.FORWARD_OFFSET * Math.cos(curpose.heading())},
                        {curpose.y() + CVConstants.FORWARD_OFFSET * sin(curpose.heading())},
                        {CVConstants.CAMERA_HEIGHT}
                }
        );
        double x = cam.get(0, 0) + absoluteRay.get(0, 0) * dist;
        double y = cam.get(1, 0) + absoluteRay.get(1, 0) * dist;
        return new Pose(x, y, 0);
    }

    private ArrayList<Pose> sortBalls() {
        ArrayList<Pose> balls = getBalls(simulation);
        Pose po = simulation ? simBotPose : follower.pose();
        balls.removeIf(ball -> isRed ? ball.x() > 70 : ball.x() < 70);
        ArrayList<Pose> best = new ArrayList<>(algo.costFunc(balls, po));
        return algo.combiner(best, po);
    }


    public Path pathGen() {
        ArrayList<Pose> ballPoses = new ArrayList<>();
        double endHeading = follower.pose().heading() - PI / 2;
        Pose addPose = new Pose(RIGHT_OFFSET * cos(endHeading), RIGHT_OFFSET * sin(endHeading));
        ballPoses.add(follower.pose());
        ballPoses.addAll(sortBalls());
        for (int i = 1; i < ballPoses.size(); i++) {
            Pose newPose = ballPoses.get(i).plus(addPose);
            ballPoses.set(i, newPose);
        }
        if (ballPoses.size() < 2) {
            return null;
        }
        Pose[] balls = ballPoses.toArray(new Pose[0]);
        drawChosen(ballPoses);
        return Paths.through(balls).tangent();
    }

    public ArrayList<Pose> pathGenSim() {
        ArrayList<Pose> ballPoses = new ArrayList<>();
        ballPoses.add(simBotPose);
        ballPoses.addAll(sortBalls());
        if (ballPoses.size() < 2) {
            return null;
        }
        Pose[] balls = ballPoses.toArray(new Pose[0]);
        Path p = Paths.through(balls).tangent();
        return ballPoses;
    }

    public void drawChosen(ArrayList<Pose> balls) {
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
            if (i == 0) {
            } else {
            }

        }
    }

    public void drawAll(ArrayList<Pose> balls) {
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
        }
    }
}
