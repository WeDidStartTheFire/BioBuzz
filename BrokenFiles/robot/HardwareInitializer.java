package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.Hardware;

public class HardwareInitializer {
    /**
     * Safely retrieves the (first) device with the indicated name which is also an instance of the
     * indicated class or interface. If no such device is found, null is returned.
     *
     * @param hardwareMap      hardware map of the robot
     * @param classOrInterface the class or interface indicating the type of the device object to be
     *                         retrieved
     * @param deviceName       the name of the device object to be retrieved
     * @return the requested device or null if not present
     */
    public static @Nullable <T> T init(@NonNull HardwareMap hardwareMap,
                                       @NonNull Class<T> classOrInterface,
                                       @NonNull String deviceName) {
        T device;
        try {
            device = hardwareMap.get(classOrInterface, deviceName);
        } catch (IllegalArgumentException e) { // Error means the device wasn't connected
            device = null;
        }
        return device;
    }

    public static @Nullable <T> T init(@NonNull HardwareMap hardwareMap, @Nullable TelemetryUtils tm,
                                       @NonNull Hardware hardware) {
        try {
            @SuppressWarnings("unchecked")
            T device = (T) hardwareMap.get(hardware.classOrInterface, hardware.deviceName);
            return device;
        } catch (IllegalArgumentException e) { // Error means the device wasn't connected
            if (tm != null) tm.warn(hardware.errorLevel, hardware.getWarnMessage());
            return null;
        }
    }
}
