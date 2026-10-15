package org.firstinspires.ftc.teamcode.teleops.tests;

import static org.firstinspires.ftc.teamcode.MatchContext.Mode.TELEOP;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.INFO;
import static org.firstinspires.ftc.teamcode.TelemetryUtils.PrintLevel.VERBOSE;
import static org.firstinspires.ftc.teamcode.enums.Color.BLUE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MatchContext;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.controllers.TeleOpController;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dev.anygeneric.blazeftc.BlazeDummyPlug;
import dev.anygeneric.blazeftc.BlazeFTC;
import dev.anygeneric.blazeftc.Hub;

@TeleOp(name = "BlazeFTC Example", group = "C")
public class TeleOp_BlazeFTC extends OpMode {
    private TeleOpController teleop;

    @Override
    public void init() {
        BlazeDummyPlug.initializeBlazeFTC(hardwareMap);
        BlazeDummyPlug.engageMotorAccel(hardwareMap);
        Robot robot = new Robot(hardwareMap, telemetry, new MatchContext(TELEOP, BLUE));
        teleop = new TeleOpController(robot, gamepad1, gamepad2);

        TelemetryUtils tm = robot.tm();
        if (!robot.drivetrain.isPoseValid())
            tm.warn(TelemetryUtils.ErrorLevel.MEDIUM, "Robot Centric driving will be used until " +
                    "the position is reset");
        else tm.print("Field Centric Driving", "✅", INFO);
        tm.print("Color", "🟦🟦Blue🟦🟦", INFO);

        ElapsedTime elt = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        BlazeDummyPlug.engageBulkReadAcceleration(hardwareMap, Hub.ExHub, 1, () -> {
            // Every time this function is called, you should have new encoder data available in
            // your motors. Run PID loops here. I recommend doing it like this, report your data do
            // not do computation here:
            robot.launcher.calculate();
            robot.turret.calculate();
            robot.intake.calculate();
            robot.writeExpansionHubMotors();
            tm.print("bulk loop time (ms)", elt.milliseconds(), VERBOSE);
            elt.reset();
            return null;
        });

//        ElapsedTime elt2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        Pedro3SingleDataLocalizer.setup(follower, () -> { // Can't use if localizer isn't Pinpoint
//            tm.print("pedro loop time (ms)", elt2.milliseconds(), VERBOSE);
//            elt2.reset();
//            tm.print(follower.pose(), VERBOSE);
//            if (follower.currentPath() != pathToFollow) follower.follow(pathToFollow);
//            follower.update();
//        });
    }

    @Override
    public void start() {
        BlazeFTC.run(0);
        teleop.start();
    }

    @Override
    public void loop() {
        teleop.drivetrainLogic();
        teleop.updateIntake();
        teleop.updateLauncherTeleOp();
        teleop.update();
    }

    @Override
    public void stop() {
        BlazeDummyPlug.closeBlazeFTC();
    }
}