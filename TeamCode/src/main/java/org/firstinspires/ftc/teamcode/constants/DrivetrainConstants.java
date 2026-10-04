package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class DrivetrainConstants {
    public static final IMU.Parameters IMU_PARAMS = new IMU.Parameters(
        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
}
