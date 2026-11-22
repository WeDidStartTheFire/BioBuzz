package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.groups.Groups.sequential;

import static java.lang.Math.toRadians;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.autos.paths.Paths_Ball_Intake;
import org.firstinspires.ftc.teamcode.enums.Color;
import org.firstinspires.ftc.teamcode.hivevision.CVPathGen;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Ball Intake", group = "A")
public class Auto_Ball_Intake extends BaseAuto2<Paths_Ball_Intake> {
    public final String baseName = "Ball Intake";
    public final String group = "A";

    private CVPathGen pather;

    @Override
    protected void onInit() {
        pather = new CVPathGen(hardwareMap, follower, false, telemetry);
    }

    private Command searchForBalls(CVPathGen makePath) {
        AtomicInteger intakenPollen = new AtomicInteger();
        AtomicInteger turnAmount = new AtomicInteger();
        return Command.build()
                .setStart(() -> {
                    intakenPollen.set(0);
                    turnAmount.set(0);
                })
                .setExecute(() -> {
                    if (!follower.isBusy() && intakenPollen.get() < 4) {
                        Path path = pather.pathGen();
                        if (path != null) {
                            follower.follow(path);
                            intakenPollen.getAndIncrement();
                        } else if (turnAmount.get() < 8){
                            Pose pose = follower.pose();
                            Path path1 = Paths.line(pose, pose.plus(new Pose(.000001, .000001)))
                                    .linear(pose.heading(), pose.heading() - toRadians(45));
                            follower.follow(path1);
                            turnAmount.getAndIncrement();
                        } else {
                            turnAmount.set(0);
                            intakenPollen.addAndGet(4);
                        }
                    }
                })
                .setDone(() -> intakenPollen.get() >= 4 && !follower.isBusy())
                .setEnd(endCondition -> {
                    intakenPollen.set(0);
                    turnAmount.set(0);
                });
    }

    @Override
    protected Command getAutoRoutine() {
        return sequential(

        );
    }

    @Override
    protected void configure() {
        super.configure(Paths_Ball_Intake.class, Color.BLUE, baseName);
    }
}
