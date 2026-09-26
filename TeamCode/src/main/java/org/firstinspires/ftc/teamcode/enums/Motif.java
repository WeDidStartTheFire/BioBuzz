package org.firstinspires.ftc.teamcode.enums;

import androidx.annotation.NonNull;

public enum Motif {
    GPP, PPG, PGP, UNKNOWN;

    @NonNull
    @Override
    public String toString() {
        switch (this) {
            case GPP:
                return "🟢🟣🟣";
            case PGP:
                return "🟣🟢🟣";
            case PPG:
                return "🟣🟣🟢";
            case UNKNOWN:
                return "Not Detected ⚫⚫⚫";
            default:
                return super.toString();
        }
    }

    /**
     * Gets the nth artifact in this motif sequence (0-indexed).
     *
     * @param n The position in the motif sequence (0-based, wraps around)
     * @return The artifact at the specified position
     */
    public Artifact getNthArtifact(int n) {
        n %= 3;
        switch (this) {
            case GPP:
                switch (n) {
                    case 0:
                        return Artifact.GREEN;
                    case 1:
                    case 2:
                        return Artifact.PURPLE;
                }
                break;
            case PGP:
                switch (n) {
                    case 1:
                        return Artifact.GREEN;
                    case 0:
                    case 2:
                        return Artifact.PURPLE;
                }
                break;
            case PPG:
                switch (n) {
                    case 2:
                        return Artifact.GREEN;
                    case 1:
                    case 0:
                        return Artifact.PURPLE;
                }
                break;
        }
        return Artifact.UNKNOWN;
    }
}
