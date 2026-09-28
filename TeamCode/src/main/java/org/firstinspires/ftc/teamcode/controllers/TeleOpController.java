package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.constants.ResetConstants.HARD_RESET_WAIT;
import static org.firstinspires.ftc.teamcode.constants.ResetConstants.SOFT_RESET_WAIT;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.LEDColors;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.LED;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Turret;

public class TeleOpController {
    private final Gamepad gamepad1, gamepad2;
    private final IntakeController intakeController;
    private final DriveController driveController;
    private final Robot robot;
    private final Follower follower;
    private final TelemetryUtils tm;
    private long lastUpdateTime;
    private int totalMs;
    private int totalUpdates;
    private final Timer softZeroTimer = new Timer(), hardZeroTimer = new Timer();
    private boolean softResetDone = false, hardResetDone = false;
    private boolean robotCentric = false;

    /**
     * Initializes the TeleOpController with robot hardware and gamepads. To be called in the init()
     * method of the OpMode.
     *
     * @param robot    Robot instance containing all hardware mechanisms
     * @param gamepad1 Primary gamepad for drivetrain control
     * @param gamepad2 Secondary gamepad for launcher/intake control
     */
    public TeleOpController(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.robot.initBulkCache();
        intakeController = new IntakeController(robot);
        driveController = new DriveController(robot, robot.context());
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        follower = robot.drivetrain.follower;
        tm = robot.drivetrain.tm;
    }

    /**
     * Initializes robot systems for TeleOp mode and starts the limelight. To be called in the
     * start() method of the OpMode.
     */
    public void start() {
        robot.limelight.start();
    }

    /**
     * Updates all robot systems including telemetry, odometry, and mechanisms.
     * Should be called repeatedly during TeleOp operation.
     */
    public void update() {
        long t = System.nanoTime();
        int ms = 0;
        if (lastUpdateTime != 0) {
            ms = Math.toIntExact((t - lastUpdateTime) / 1_000_000);
            totalMs += ms;
            totalUpdates++;
            tm.print("dt (ms)", ms);
            tm.print("avg dt (ms)", totalMs / totalUpdates);
        }
        lastUpdateTime = t;
        robot.updateBulkCache();
        robot.drivetrain.update();
        if (gamepad2.rightStickButtonWasPressed()) {
            robot.turret.setTarget(Turret.Target.MANUAL);
            robot.turret.toggleTargetChangeable();
        }
        robot.turret.rotateManual(gamepad2.right_stick_x * .001 * ms);
        robot.turret.update(true);
        robot.led.update();
        tm.updateOnlyPanels(10);
    }

    /**
     * Stops all robot systems and displays final logs.
     * Should be called when TeleOp mode ends.
     */
    public void stop() {
        driveController.stop();
        intakeController.stop();
        robot.limelight.stop();
        tm.showLogs();
        tm.update();
    }

    /**
     * Logic for the drivetrain during TeleOp. Defaults to using field centric driving if current
     * position is valid. Defaults to using Pedro Pathing for movement.
     */
    public void drivetrainLogic() {
        drivetrainLogic(robot.drivetrain.isPoseValid());
    }

    /**
     * Logic for the drivetrain during TeleOp. Defaults to using Pedro Pathing for movement.
     *
     * @param fieldCentric Whether to use field centric driving
     */
    public void drivetrainLogic(boolean fieldCentric) {
        drivetrainLogic(fieldCentric, true);
    }

    /**
     * Logic for the drivetrain during TeleOp
     *
     * @param fieldCentric Whether to use field centric driving
     * @param usePedro     Whether to use Pedro Pathing
     */
    public void drivetrainLogic(boolean fieldCentric, boolean usePedro) {
        if (robot.drivetrain.isPoseValid() && usePedro) {
//            if (gamepad1.xWasPressed())
//                driveController.follow(robot.context().alliance() == BLUE ? BLUE_FAR_LAUNCH : RED_FAR_LAUNCH);
//            if (gamepad1.aWasPressed())
//                driveController.follow(robot.context().alliance() == BLUE ? BLUE_HUMAN_PLAYER : RED_HUMAN_PLAYER);
//            if (gamepad1.yWasPressed())
//                driveController.follow(robot.context().alliance() == BLUE ? BLUE_BASE_ZONE : RED_BASE_ZONE);
            if (gamepad1.bWasPressed()) driveController.toggleAiming();
        }
        if (gamepad1.dpadDownWasPressed()) {
            softZeroTimer.resetTimer();
            softResetDone = false;
        }
        if (gamepad1.dpad_down) {
            robot.led.setColor(LEDColors.GREEN, LED.Priority.CRITICAL);
            if (softZeroTimer.getElapsedTimeSeconds() < SOFT_RESET_WAIT)
                robot.led.setColor(LEDColors.WHITE, LED.Priority.CRITICAL);
            else if (!softResetDone) {
                softResetDone = driveController.softReset();
                if (!softResetDone)
                    robot.led.setColor(LEDColors.RED, LED.Priority.CRITICAL);
            }
        }
        if (gamepad1.dpadUpWasPressed()) {
            hardZeroTimer.resetTimer();
            hardResetDone = false;
        }
        if (gamepad1.dpad_up) {
            robot.led.setColor(LEDColors.GREEN, LED.Priority.CRITICAL);
            if (hardZeroTimer.getElapsedTimeSeconds() < HARD_RESET_WAIT)
                robot.led.setColor(LEDColors.WHITE, LED.Priority.CRITICAL);
            else if (!hardResetDone) {
                hardResetDone = true;
                driveController.hardReset();
            }
        }
        if (gamepad1.dpadLeftWasPressed()) robotCentric = true;
        else if (gamepad1.dpadRightWasPressed()) robotCentric = false;
        fieldCentric = fieldCentric && !robotCentric;
        tm.print("Robot Centric", robotCentric);
        tm.print("Field Centric", fieldCentric);
        tm.drawRobot(follower, 250);
        if (robot.drivetrain.getPose() != null) tm.print(robot.drivetrain.getPose());
        if (usePedro) driveController.updateTeleOp(gamepad1, fieldCentric);
        else driveController.updateTeleOpNoPedro(gamepad1, fieldCentric);
    }

    /**
     * Handles intake control based on gamepad input.
     * Manages intake, outtake, and automatic stopping.
     */
    public void updateIntake() {
        if (gamepad1.right_trigger > 0.3) intakeController.intake();
        else if (gamepad1.right_bumper) intakeController.outtake();
        else if (gamepad1.left_bumper) intakeController.manualIntake();
        else if (intakeController.isBusy()) intakeController.stop();
        intakeController.update();
        // Stops innerIntake if it isn't called by the next call to updateIntake()
        intakeController.stopInnerIntake();
    }


    /**
     * Updates launcher controls during TeleOp mode.
     * Handles manual spin, intake, and launching.
     */
    public void updateLauncherTeleOp() {
        if (gamepad2.dpadUpWasPressed()) robot.launcher.increaseVelocityModifier(25);
        if (gamepad2.dpadDownWasPressed()) robot.launcher.increaseVelocityModifier(-25);

        tm.print("Launcher Vel", robot.launcher.getCachedVel());
        tm.print("Goal", robot.launcher.getGoalVel());
    }
}
