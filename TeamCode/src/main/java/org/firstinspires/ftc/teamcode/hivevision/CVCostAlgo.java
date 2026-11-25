package org.firstinspires.ftc.teamcode.hivevision;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class CVCostAlgo {
    private ArrayList<ArrayList<Pose>> permutationsGen(ArrayList<Pose> ballList,
                                                       ArrayList<Pose> buffer,
                                                       ArrayList<ArrayList<Pose>> candidates,
                                                       int index) {

        if (buffer.size() == Math.min(4, ballList.size())) {
            candidates.add(new ArrayList<>(buffer));
            return candidates;
        }

        for (int i = index; i < ballList.size(); i++) {
            buffer.add(ballList.get(i));

            permutationsGen(ballList, buffer, candidates, i + 1);

            buffer.remove(buffer.size() - 1);
        }
        return candidates;
    }

    public ArrayList<Pose> costFunc(ArrayList<Pose> balls, Pose curpose) {
        if (balls.isEmpty()) {
            return new ArrayList<>();
        }
        ArrayList<ArrayList<Pose>> canidateGroups =
                new ArrayList<>(permutationsGen(balls, new ArrayList<>(), new ArrayList<>(), 0));
        ArrayList<ArrayList<Pose>> candidateRoutes = new ArrayList<>();

        for (ArrayList<Pose> group : canidateGroups) {
            ArrayList<Pose> sorted = new ArrayList<>();
            group.sort(Comparator.comparingDouble(curpose::distance));
            sorted.add(group.remove(0));
            while (!group.isEmpty()) {
                group.sort(Comparator.comparingDouble(ball ->
                        sorted.get(sorted.size() - 1).distance(ball)));
                sorted.add(group.remove(0));
            }
            candidateRoutes.add(sorted);
        }
        ArrayList<Routes> routes = new ArrayList<>();
        for (ArrayList<Pose> group : candidateRoutes) {
            ArrayList<Pose> route = new ArrayList<>(group.subList(0, Math.min(group.size(), 4)));
            double dist = curpose.distance(route.get(0));
            for (int i = 0; i < route.size() - 1; i++) {
                dist += route.get(i).distance(route.get(i + 1));
            }
            routes.add(new Routes(route, dist));
        }

        routes.sort(Comparator.comparingDouble(route -> route.score));
        Collections.reverse(routes);

        return wallPadding(routes.get(0).balls, curpose);


    }

    public ArrayList<Pose> wallPadding(ArrayList<Pose> balls, Pose curPose) {
        ArrayList<Pose> bordered = new ArrayList<>();
        double padding = 7;
        for (Pose ball : balls) {
            if (ball.x() <= padding) {
                ball = ball.withX(padding);
            } else if (ball.x() >= 141.5 - padding) {
                ball = ball.withX(141.5 - padding);
            }
            if (ball.y() <= 7) {
                ball = ball.withY(padding);
            } else if (ball.y() >= 141.5 - padding) {
                ball = ball.withY(141.5 - padding);
            }
            bordered.add(ball);
        }
        return bordered;
    }

    public ArrayList<Pose> combiner(ArrayList<Pose> route, Pose curPose) {
        if (route.size() <= 1) return route;
        for (int i = 0; i < route.size() - 1; i++) {
            if (route.get(i).distance(route.get(i + 1)) > 16) continue;

            Vector2D ballToBall = route.get(i + 1).toVector2D().minus(route.get(i).toVector2D());

            double theta;
            if (i == 0) {
                theta = Math.atan2(route.get(i).y() - curPose.y(), route.get(i).x() - curPose.x());
            } else {
                theta = Math.atan2(route.get(i).y() - route.get(i - 1).y(), route.get(i).x() - route.get(i - 1).x());
            }


            Vector2D botVec = Vector2D.polar(1, theta + Math.PI / 2.0);
            double compx = botVec.dot(ballToBall);

            Vector2D botVecPerp = Vector2D.polar(1, theta);
            double compy = botVecPerp.dot(ballToBall);
            if (Math.abs(compx) < 10 && Math.abs(compy) < 6) {
                double x = (route.get(i).x() + route.get(i + 1).x()) / 2.0;
                double y = (route.get(i).y() + route.get(i + 1).y()) / 2.0;
                route.set(i, new Pose(x, y));
                route.remove(i + 1);
            }
        }
        return route;
    }

    static class Routes {
        ArrayList<Pose> balls;
        double distance;
        double score;

        Routes(ArrayList<Pose> balls, double distance) {
            this.balls = balls;
            this.distance = distance;
            this.score = balls.size() / distance;
        }
    }
}
