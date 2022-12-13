package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;

/**
 * Simple PID controller with simple Feedforward coefficient.
 */
//@Config
public class PIDFMotorController {

    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    private double period;

    private boolean continuous = false;
    private double maximumInput;
    private double minimumInput;
    private double maximumIntegral = 1.0;
    private double minimumIntegral = -1.0;

    private double error;
    private double prevError;
    private double acculumatedError;
    private double direction;

    public PIDFMotorController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0.02);
    }

    public PIDFMotorController(double kP, double kI, double kD, double kF, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.period = period;

        reset();
    }

    public double calculate(double measurement, double setpoint) {

        // Note previous error
        prevError = error;

        // Calculate error
        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            error = InfiniteMath.inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }


        // Calculate direction for feedforward gain
        direction = Math.signum(error);

        // Calculate derivative
        double derivative = (error - prevError) / period;

        // Calculate integral
        if(kI != 0d)
            acculumatedError = InfiniteMath.clamp(acculumatedError + (error * period), minimumIntegral / kI, maximumIntegral / kI);
        double integral = acculumatedError;

        // Calculate output from error and gains
        return (kP * error) + (kD * derivative) + (kI * integral) + (kF * direction);

    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public void reset() {
        error = 0;
        prevError = 0;
        acculumatedError = 0;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

}