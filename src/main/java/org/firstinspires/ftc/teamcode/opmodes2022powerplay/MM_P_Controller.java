package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_P_Controller {
    private final LinearOpMode opMode;

    private ElapsedTime dTime = new ElapsedTime();

    private double setpoint = 0;
    private double minInput = 0;
    private double maxInput = 0;
    private double minOutput = 0;
    private double maxOutput = 0;
    private double outputRange = 0;
    private double inputRange = 0;
    private double currentError = 0;
    private double lastError = 0;
    private double currentTime = 0;
    private double lastTime = 0;
    private double currentInput = 0;

    private final double PCT_THRESHOLD;
    private final double P_COEFFICIENT;
    private final double D_COEFFICIENT;

    public MM_P_Controller(LinearOpMode opMode, double pctThreshold, double pCoefficient){
        this.opMode = opMode;

        PCT_THRESHOLD = pctThreshold;
        P_COEFFICIENT = pCoefficient;
        D_COEFFICIENT = 0.00001698;
    }

    public double calculatePower(double currentInput){
        this.currentInput = currentInput;
        currentError = setpoint - currentInput;

        double power = ((P_COEFFICIENT * currentError) + getD()) * outputRange;
        dTime.reset();
        if(power > outputRange){
            power = outputRange;
        }
        if (currentError < 0) {
            power = power - minOutput;
        } else {
            power = power + minOutput;
        }
        opMode.telemetry.addData("input", currentInput);
        opMode.telemetry.addData("input range", inputRange);
        opMode.telemetry.addData("current error", Math.abs(currentError));
        opMode.telemetry.addData("calculated power", power);
        lastError = currentError;
        currentTime = dTime.seconds();
        lastTime = currentTime;
        return power;
    }

    public boolean reachedTarget(){
        return (Math.abs(currentError / inputRange) * 100) < PCT_THRESHOLD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        inputRange = maxInput - minInput;
    }

    public void setup(double start, double target) {
        setSetpoint(target);
        setInputRange(start, target);
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        outputRange = maxOutput - minOutput;
    }

    public double getCurrentError(){
        return currentError;
    }

    public double getCurrentInput(){
        return currentInput;
    }

    private double getD() {
        if (lastTime != 0) {
            return (D_COEFFICIENT * ((currentError - lastError) / (dTime.seconds())));
        } else {
            return 0;
        }
    }
}