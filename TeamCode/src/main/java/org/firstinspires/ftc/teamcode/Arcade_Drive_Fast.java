package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcade Drive Fast")
public class Arcade_Drive_Fast extends LinearOpMode {
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor armExtender;
    private DcMotor armLander;
    private Servo markerReleaser;
    private BNO055IMU armImu;
    private DcMotor intakeMotor;
    private boolean isMovingToGoal = false;

    private boolean armHasBeenHolding = false;

    private PID_Controller armHoldP = new PID_Controller(0.006, 0.0, 0.0);
    private PID_Controller armHoldD = new PID_Controller(0.0, 0.0, 0.001);
    private boolean useP = true;

    private double armRotatorPower = 0.0;

    private Sensor_Thread sensorThread;

    //Maps robot parts to data values in config file, sets up opMode
    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive = hardwareMap.get(DcMotor.class, "brd");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        armExtender = hardwareMap.get(DcMotor.class, "ae");
        armLander = hardwareMap.get(DcMotor.class, "al");
        markerReleaser = hardwareMap.get(Servo.class, "mr");
        intakeMotor = hardwareMap.get(DcMotor.class, "im");
        armImu = hardwareMap.get(BNO055IMU.class, "arm_imu");
        DcMotor armRotator = hardwareMap.get(DcMotor.class, "ar");

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armExtender.setDirection(DcMotor.Direction.FORWARD);
        armLander.setDirection(DcMotor.Direction.FORWARD);
        markerReleaser.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        setupImu();

        ArmTargetDirection.setImu(armImu);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        markerReleaser.setPosition(-1.0);
        teleopTurnPID.resetPID();
        teleopTurnPID.setSetpoint(0.0);

        sensorThread = new Sensor_Thread(armRotator);
        sensorThread.start();

        while (opModeIsActive()) {
            run();
        }
    }
    private boolean driveSlowFactor = false;
    private PID_Controller teleopTurnPID = new PID_Controller(0.012, 0.0, 0.0013);

    private long lastCheckedTime = 0;

    private boolean lastY = false;
    private boolean lastX = false;

    private long lastTime = 0;

    //Sets each individual drive's power based on forward, turn, and strafe  inputs
    private void setDrive(double forwardPower, double turnPower, double strafePower) {
        forwardPower = Range.clip(forwardPower, -1.0, 1.0);
        turnPower = Range.clip(turnPower, -1.0, 1.0);
        strafePower = Range.clip(strafePower, -1.0, 1.0);
        backLeftDrive.setPower(forwardPower + turnPower - strafePower);
        backRightDrive.setPower(forwardPower - turnPower + strafePower);
        frontLeftDrive.setPower(forwardPower + turnPower + strafePower);
        frontRightDrive.setPower(forwardPower - turnPower - strafePower);
    }

    private void setArmRotator(double armPower) {
        armHasBeenHolding = false;
        isMovingToGoal = false;

        armRotatorPower = armPower;

        double k_GRAVITY = 0.2;
        armPower += k_GRAVITY * Math.cos(Math.toRadians(sensorThread.getArmRotatorPosition()));

        sensorThread.setArmRotatorPower(armPower);
    }

    //Sets power of armExtender
    private void setArmExtender(double extendPower) {
        armExtender.setPower(extendPower);
    }

    //Inits IMU and sets up its configuation and parameters
    private void setupImu() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;

        armImu.initialize(imuParameters);
    }

    //Sets power of armLander
    private void setArmLander(double liftPower) {
        armLander.setPower(liftPower);
    }

    private void setIntakeMotor(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    private void holdArmPosition() {
        holdArmPosition(sensorThread.getArmRotatorPosition());
    }

    private void holdArmPosition(double armPositionToHold) {
        if (!armHasBeenHolding) {
            armHoldP.setSetpoint(armPositionToHold);
            armHoldP.resetPID();
            armHoldD.setSetpoint(armPositionToHold);
            armHoldD.resetPID();
        }
        // Always use the derivative controller
        double power = armHoldD.update(sensorThread.getArmRotatorPosition());

        if (useP)
            // If using the proportional controller, add the proportional component
            power += armHoldP.update(sensorThread.getArmRotatorPosition());
        else
            // If NOT using the proportional controller, at least keep the controller updated
            armHoldP.update(sensorThread.getArmRotatorPosition());
        setArmRotator(power);
        armHasBeenHolding = true;
    }
    private double mapJoyStick(float joyStickInput){
        double output = (double)joyStickInput;
        output = Math.pow(output, 3.0);
        return output;
    }

    private void setArmRotatorGoal(double goalPower) {
        useP = false;

        if (!isMovingToGoal)
            lastCheckedTime = System.currentTimeMillis();

        long newCheckedTime = System.currentTimeMillis();

        double increment = (double) (newCheckedTime - lastCheckedTime);
        double ARM_POWER_PER_MS_SPEED_UP = 0.0013;
        double ARM_POWER_PER_MS_SPEED_DOWN = 0.01;
        if (Math.abs(goalPower) > Math.abs(armRotatorPower))
            increment *= ARM_POWER_PER_MS_SPEED_UP;
        else
            increment *= ARM_POWER_PER_MS_SPEED_DOWN;

        if (goalPower > armRotatorPower)
            armRotatorPower += increment;
        else
            armRotatorPower -= increment;

        if (Math.abs(armRotatorPower - goalPower) < 0.1)
            armRotatorPower = goalPower;

        double ARM_ROTATOR_MINIMUM = 0.2;
        if (goalPower > ARM_ROTATOR_MINIMUM && armRotatorPower < ARM_ROTATOR_MINIMUM)
            armRotatorPower = ARM_ROTATOR_MINIMUM;
        if (goalPower < -ARM_ROTATOR_MINIMUM && armRotatorPower > -ARM_ROTATOR_MINIMUM)
            armRotatorPower = -ARM_ROTATOR_MINIMUM;
        armRotatorPower = Range.clip(armRotatorPower, -0.75, 0.75);

        setArmRotator(armRotatorPower);
        lastCheckedTime = newCheckedTime;
        isMovingToGoal = true;
    }

    //Begins teleop
    private void run() {
        timeIt("Loop Around");

        markerReleaser.setPosition(-1.0);
        double p2_MULT = 0.3;
        double forwardPower = mapJoyStick(-gamepad1.left_stick_y) + mapJoyStick(gamepad2.left_stick_x) * p2_MULT;
        double turnPower = mapJoyStick(gamepad1.right_stick_x) + mapJoyStick(gamepad2.right_stick_x) * p2_MULT;
        double strafePower = mapJoyStick(gamepad1.left_stick_x) + mapJoyStick(gamepad2.left_stick_y) * p2_MULT;

        double SLOW_TURN_MULT = 0.3;
        if (sensorThread.getArmRotatorPosition() > 140.0)
            turnPower *= SLOW_TURN_MULT;

        if (Math.abs(strafePower) > Math.abs(forwardPower))
            forwardPower = 0.0;
        else
            strafePower = 0.0;

        //Slows the drive by a certain factor if true
        if (driveSlowFactor) {
            double SLOW_DRIVE_SCALE_FACTOR = 0.5;
            forwardPower *= SLOW_DRIVE_SCALE_FACTOR;
            strafePower *= SLOW_DRIVE_SCALE_FACTOR;
        }
        setDrive(forwardPower, turnPower, strafePower);

        boolean verboseTiming = true;
        if (verboseTiming)
            timeIt("Drive");

        boolean yButton = gamepad1.y;
        boolean xButton = gamepad1.x;

        boolean yIsPressed = yButton && !lastY;
        boolean xIsPressed = xButton && !lastX;

        lastY = yButton;
        lastX = xButton;


        //Lifts the arm to certain positions and maps them to certain joystick positions
        if (gamepad1.left_bumper) {
            setArmRotatorGoal(1.0);
        } else if (gamepad1.left_trigger > 0.1f) {
            setArmRotatorGoal(-1.0);
        } else if (yIsPressed) { // Up
            armHasBeenHolding = false;
            useP = true;
            holdArmPosition(90.0);
        } else if (xIsPressed) { // Center
            armHasBeenHolding = false;
            useP = true;
            holdArmPosition(190.0);
        } else {
            holdArmPosition();
        }

        if (verboseTiming)
            timeIt("Arm Rotator");

        //Extends the arm to certain positions, and maps them to certain joystick positions
        if (gamepad1.right_bumper) {
            double ARM_EXTENDER_POWER_UP = 1.0;
            setArmExtender(ARM_EXTENDER_POWER_UP);
        } else if (gamepad1.right_trigger > 0.1f) {
            double ARM_EXTENDER_POWER_DOWN = -1.0;
            setArmExtender(ARM_EXTENDER_POWER_DOWN);
        } else {
            double ARM_EXTENDER_POWER_IDLE = 0.0;
            setArmExtender(ARM_EXTENDER_POWER_IDLE);
        }

        //Set lander to certain positions, and maps them to certain joystick positions
        if (gamepad2.y) {
            double ARM_LANDER_POWER_UP = 1.0;
            setArmLander(ARM_LANDER_POWER_UP);
        } else if ((gamepad2.a) && (!gamepad1.start && !gamepad2.start)) {
            double ARM_LANDER_POWER_DOWN = -1.0;
            setArmLander(ARM_LANDER_POWER_DOWN);
        } else {
            double ARM_LANDER_POWER_IDLE = 0.0;
            setArmLander(ARM_LANDER_POWER_IDLE);
        }

        if (verboseTiming)
            timeIt("Arm Extender and Lander");

        //Enables or disables a slower drive
        if (gamepad1.dpad_left) {
            driveSlowFactor = true;
        } else if (gamepad1.dpad_right) {
            driveSlowFactor = false;
        }

        double INTAKE_POWER_END = -1.0;
        if (gamepad2.left_trigger > 0.1f) {
            setIntakeMotor(INTAKE_POWER_END);
        } else if (gamepad2.left_bumper) {
            setIntakeMotor(-INTAKE_POWER_END);
        } else {
            setIntakeMotor(0.0);
        }

        if (verboseTiming)
            timeIt("Slow Drive and Intake");

        telemetry.addData("Arm Position", sensorThread.getArmRotatorPosition());
        telemetry.addData("Arm Motor Power", sensorThread.getArmRotatorPower());
        telemetry.addData("Setpoint", armHoldP.getSetpoint());
        telemetry.update();
    }

    private void timeIt(String message)
    {
        long time = System.currentTimeMillis();
        telemetry.addData(message,time - lastTime);
        lastTime = time;
    }
}