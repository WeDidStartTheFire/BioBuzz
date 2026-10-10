package org.firstinspires.ftc.teamcode.enums;

public enum LEDColors {
    OFF, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET, WHITE;

    /**
     * Gets the position value for this LED color on the LED strip. <p>
     * Based on <a href="https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1">this image</a>.
     *
     * @return Position value from 0.0 to 1.0 representing the color's position on the LED strip
     */
    public double position() {
        switch (this) {
            case RED:
                return 0.277;
            case ORANGE:
                return 0.333;
            case YELLOW:
                return 0.388;
            case SAGE:
                return 0.444;
            case GREEN:
                return 0.500;
            case AZURE:
                return 0.555;
            case BLUE:
                return 0.611;
            case INDIGO:
                return 0.666;
            case VIOLET:
                return 0.722;
            case WHITE:
                return 1.0;
            case OFF:
            default:
                return 0.0;
        }
    }
}
