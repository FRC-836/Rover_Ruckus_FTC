package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public abstract class Teleop_Parent extends Robot_Parent {
    //Final and boolean powers which are used to set powers of specific parts of the robot
    protected boolean driveSlowFactor = false;
    protected final double SLOW_DRIVE_SCALE_FACTOR = 0.5;
    protected final double ARM_LANDER_POWER_UP = 1.0;
    protected final double ARM_LANDER_POWER_DOWN = -1.0;
    protected final double ARM_LANDER_POWER_IDLE = 0.0;
    protected final double ARM_EXTENDER_POWER_UP = 1.0;
    protected final double ARM_EXTENDER_POWER_DOWN = -1.0;
    protected final double ARM_EXTENDER_POWER_IDLE = 0.0;
    protected final double ARM_ROTATOR_POWER_UP = 1.0;
    protected final double ARM_ROTATOR_POWER_DOWN = -1.0;
    protected final double ARM_ROTATOR_POWER_IDLE = 0.0;
    protected final double INTAKE_POWER_END = -1.0;
    protected final double INTAKE_SHIFTER_POWER = 0.6;
    private final double ARM_POWER_PER_MS_SPEED_UP = 0.0013;
    private final double ARM_POWER_PER_MS_SPEED_DOWN = 0.01;
    private final double ARM_ROTATOR_MINIMUM = 0.2;
    protected PID_Controller teleopTurnPID = new PID_Controller(0.012, 0.0, 0.0013);
    protected TargetDirection currentFacingDirection;

    private double armRotatorGoal = 0.0;
    private long lastCheckedTime = 0;

    //Called on init
    @Override
    public void getReady() {
        isAuto = false;
    }

    //Called on start, does actions when the robot is running
    @Override
    public void go() {
        begin();
        setMarkerReleaser(-1.0);
        teleopTurnPID.setSetpoint(0.0);
        while (opModeIsActive()) {
            run();
        }
    }

    abstract public void begin();

    abstract public void run();

    //Transfers float inputs of joystick values to doubles
    protected double mapJoyStick(float joyStickInput){
        return (double)joyStickInput;
    }

    protected void setArmRotatorGoal(double goalPower) {
        useP = false;
        armHoldStatus = ArmHoldStatus.HOLDING;
        armRotatorGoal = goalPower;

        if (!isMovingToGoal)
            lastCheckedTime = System.currentTimeMillis();

        long newCheckedTime = System.currentTimeMillis();

        double increment = (double) (newCheckedTime - lastCheckedTime);
        if (Math.abs(armRotatorGoal) > Math.abs(armRotatorPower))
            increment *= ARM_POWER_PER_MS_SPEED_UP;
        else
            increment *= ARM_POWER_PER_MS_SPEED_DOWN;

        if (armRotatorGoal > armRotatorPower)
            armRotatorPower += increment;
        else
            armRotatorPower -= increment;

        if (Math.abs(armRotatorPower - armRotatorGoal) < 0.1)
            armRotatorPower = armRotatorGoal;

        if (armRotatorGoal > ARM_ROTATOR_MINIMUM && armRotatorPower < ARM_ROTATOR_MINIMUM)
            armRotatorPower = ARM_ROTATOR_MINIMUM;
        if (armRotatorGoal < -ARM_ROTATOR_MINIMUM && armRotatorPower > -ARM_ROTATOR_MINIMUM)
            armRotatorPower = -ARM_ROTATOR_MINIMUM;
        armRotatorPower = Range.clip(armRotatorPower, -0.75, 0.75);

        setArmRotator(armRotatorPower);
        lastCheckedTime = newCheckedTime;
        isMovingToGoal = true;
    }

}
