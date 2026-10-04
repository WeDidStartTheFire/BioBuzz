package org.firstinspires.ftc.teamcode.pedro.controllers;

import com.pedropathing.controllers.PIDController;

public class PIDFController extends PIDController {
    public double kF;
    private double target;
    private double position;
    private double feedForwardInput;
    private boolean targetIsFeedForward = true;

    public PIDFController(PIDFCoefficients coefficients) {
        super(coefficients.kP, coefficients.kI, coefficients.kD);
        this.kF = coefficients.kF;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double target() {
        return target;
    }

    public void updatePosition(double position) {
        this.position = position;
    }

    public double getError() {
        return target - position;
    }

    public void updateFeedForwardInput(double feedForwardInput) {
        this.feedForwardInput = feedForwardInput;
        this.targetIsFeedForward = false;
    }

    public double calculate() {
        return calculate(target, target - getError());
    }

    @Override
    public double calculate(double target, double error) {
        if (targetIsFeedForward) feedForwardInput = target;
        return super.calculate(target, error) + kF * feedForwardInput;
    }

    @Override
    public double calculate(double target, double error, double velocity) {
        if (targetIsFeedForward) feedForwardInput = target;
        return super.calculate(target, error, velocity) + kF * feedForwardInput;
    }

    public void setCoefficients(PIDFCoefficients coefficients) {
        super.kP = coefficients.kP;
        super.kI = coefficients.kI;
        super.kD = coefficients.kD;
        this.kF = coefficients.kF;
    }
}
