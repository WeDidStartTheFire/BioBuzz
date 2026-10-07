package org.firstinspires.ftc.teamcode.autos.primary;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.BaseAuto2;
import org.firstinspires.ftc.teamcode.autos.paths.Paths_Test3;
import org.firstinspires.ftc.teamcode.enums.Color;

@Autonomous(name = Auto_Test3.name, group = "B")
public class Auto_Test3 extends BaseAuto2<Paths_Test3> {
    private final Color color = Color.BLUE;
    public static final String name = "Test Auto 2";
    private final Class<Paths_Test3> pathsClass = Paths_Test3.class;

    @Override
    protected Command getAutoRoutine() {
        return sequential(
                follow(follower, paths.path1()),
                follow(follower, paths.path2()),
                follow(follower, paths.path3()),
                follow(follower, paths.path4()),
                follow(follower, paths.path5()),
                infinite(() -> tm.print("Finished", INFO))
        );
    }

    @Override
    protected void configure() {
        super.configure(pathsClass, color, name, false);
    }
}
