package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.enums.Hardware.EXPANSION_HUB;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.ErrorLevel.MEDIUM;
import static org.firstinspires.ftc.teamcode.utils.TelemetryUtils.PrintLevel.DEBUG;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.enums.Hub;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import java.util.function.DoubleSupplier;

public class VoltageSensor {

    private final DoubleSupplier eHVoltage;
    private final DoubleSupplier cHVoltage;
    private double cachedEHVoltage;
    private double cachedCHVoltage;
    private boolean eHCached = false;
    private boolean cHCached = false;
    private boolean useCache = false;

    public VoltageSensor(TelemetryUtils tm, HardwareMap hardwareMap) {
        LynxModule expansionHub = HardwareInitializer.init(hardwareMap, tm, EXPANSION_HUB);
        if (expansionHub != null) {
            eHVoltage = () -> expansionHub.getInputVoltage(VoltageUnit.VOLTS);
            tm.print("Expansion Hub found via default port (for voltage sensor)", DEBUG);
        } else {
            com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor =
                    hardwareMap.voltageSensor.iterator().next();
            eHVoltage = voltageSensor::getVoltage;
            tm.warn(MEDIUM, "Failed to find expansion hub, will use default voltage sensor for " +
                    "PID adjustments");
        }

        com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor =
                hardwareMap.voltageSensor.iterator().next();
        DoubleSupplier tmpControlHubVoltage = voltageSensor::getVoltage;
        LynxModule controlHub = HardwareInitializer.init(hardwareMap, tm, EXPANSION_HUB);
        if (controlHub != null) {
            tmpControlHubVoltage = () -> controlHub.getInputVoltage(VoltageUnit.VOLTS);
            tm.print("Control Hub found via default port (for voltage sensor)", DEBUG);
        } else {
            boolean controlHubFound = false;
            for (LynxModule module : hardwareMap.getAll(LynxModule.class))
                if (module.isParent()) {
                    tmpControlHubVoltage = () -> module.getInputVoltage(VoltageUnit.VOLTS);
                    controlHubFound = true;
                    tm.print("Control Hub found via default search (for voltage sensor)", DEBUG);
                    break;
                }
            if (!controlHubFound)
                tm.print("Control Hub not found; default used (for voltage sensor)", DEBUG);
        }
        cHVoltage = tmpControlHubVoltage;
    }

    public void setUseCache(boolean useCache) {
        this.useCache = useCache;
    }

    public void clearCache() {
        eHCached = cHCached = false;
    }

    public double getVoltage(Hub hub) {
        if (hub == Hub.EXPANSION && (!useCache || !eHCached)) {
            eHCached = true;
            cachedEHVoltage = eHVoltage.getAsDouble();
        }
        if (hub == Hub.CONTROL && (!useCache || !cHCached)) {
            cHCached = true;
            cachedCHVoltage = cHVoltage.getAsDouble();
        }
        return hub == Hub.EXPANSION ? cachedEHVoltage : cachedCHVoltage;
    }
}
