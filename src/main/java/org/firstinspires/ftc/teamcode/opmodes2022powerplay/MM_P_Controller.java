package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_P_Controller {
    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private double setpoint = 0;
    private double minInput = 0;
    private double maxInput = 0;
    private double minOutput = 0;
    private double maxOutput = 0;
    private double outputRange = 0;
    private double inputRange = 0;
    private double currentError = 0;
    private double currentInput = 0;

    private final double PCT_THRESHOLD;
    private final double P_COEFFICIENT;

    public MM_P_Controller(LinearOpMode opMode, double pctThreshold, double pCoefficient){
        this.opMode = opMode;

        //defining the state of this instance
        PCT_THRESHOLD = pctThreshold;
        P_COEFFICIENT = pCoefficient;
    }
    public double calculatePower(double currentInput){
        this.currentInput = currentInput;
/*        absError = Math.abs(setpoint - currentInput);*/
        currentError = setpoint - currentInput;

        double power = currentError * P_COEFFICIENT * (outputRange);
        if(power > outputRange){
            power = outputRange;
        }
        if (power < 0) {
            power = power - minOutput;
        } else {
            power = power + minOutput;
        }
        opMode.telemetry.addData("input", currentInput);
        opMode.telemetry.addData("input range", inputRange);
        opMode.telemetry.addData("current error", Math.abs(currentError));
        opMode.telemetry.addData("calculated power", power);
        return power;
    }
    public boolean reachedTarget(){
        if ((Math.abs(currentError / inputRange) * 100) < PCT_THRESHOLD){
            return true;
        }
        return false;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        inputRange = maxInput - minInput;
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
}