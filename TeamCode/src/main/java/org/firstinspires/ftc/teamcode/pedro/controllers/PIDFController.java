package org.firstinspires.ftc.teamcode.pedro.controllers;

import com.pedropathing.controllers.PIDController;

public class PIDFController extends PIDController {
    public double kF;
    public double target;
    public double error;
    public double feedForwardInput;
    public boolean targetIsFeedForward = true;

    public PIDFController(PIDFCoefficients coefficients) {
        super(coefficients.kP, coefficients.kI, coefficients.kD);
        this.kF = coefficients.kF;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void updatePosition(double position) {
        this.error = target - position;
    }

    public void updateError(double error) {
        this.error = error;
    }

    public double getError() {
        return error;
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
        return super.calculate(target, error) + kF * target;
    }

    @Override
    public double calculate(double target, double error, double velocity) {
        return super.calculate(target, error, velocity) + kF * target;
    }

    public void setCoefficients(PIDFCoefficients coefficients) {
        super.kP = coefficients.kP;
        super.kI = coefficients.kI;
        super.kD = coefficients.kD;
        this.kF = coefficients.kF;
    }
}
