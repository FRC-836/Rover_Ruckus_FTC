package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by user on 7/10/2018.
 */
public class PID_Controller{

    ElapsedTime runtime;

    private double setpoint = 0.0;
    private double error = 0.0;
    private double lastError = 0.0;
    private double time = 0.0;
    private double lastTime = 0.0;
    private double pValue = 0.0;
    private double PGAIN;
    private double iValue = 0.0;
    private double IGAIN;
    private double dValue = 0.0;
    private double DGAIN;

    public PID_Controller(double PGAIN, double IGAIN, double DGAIN){
        this.runtime = new ElapsedTime();
        resetPID();
        this.PGAIN = PGAIN;
        this.IGAIN = IGAIN;
        this.DGAIN = DGAIN;
    }

    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
    }

    public double update(double input){
        lastError = error;
        lastTime = time;
        error = setpoint - input;
        time = runtime.seconds();
        pValue = PGAIN * error;
        iValue += IGAIN * (lastError + error) * (0.5) * (time - lastTime);
        dValue = DGAIN * (error - lastError) / (time - lastTime);
        return getPID();
    }

    public void resetPID() {
        resetPID(0.0);
    }

    public void resetPID(double startingIValue) {
        runtime.reset();
        iValue = startingIValue;
    }

    public void displayCurrentPID(Telemetry telemetry) {
        telemetry.addData("P: ",pValue);
        telemetry.addData("I: ",iValue);
        telemetry.addData("D: ",dValue);
        telemetry.addData("PID: ",getPID());
    }

    private double getPID() {
        return pValue + iValue + dValue;
    }
}