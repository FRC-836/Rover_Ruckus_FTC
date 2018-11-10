package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CodyTest")
public class Cody extends OpMode {

    private BNO055IMU imu;
    private TargetDirection target;
    private PID_Controller turnController = new PID_Controller(0.015, 0.0, 0.0);

    private boolean buttonsEnabled = true;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void start() {
        setupImu();
        TargetDirection.setupImu(0.0, imu);
        target = TargetDirection.makeTargetAtFieldPosition(90.0);

        turnController.setSetpoint(0.0);
        turnController.resetPID();
    }

    @Override
    public void loop() {
        boolean inc10 = gamepad1.dpad_up;
        boolean dec10 = gamepad1.dpad_down;
        boolean set90Field = gamepad1.x;
        boolean set90RobotRight = gamepad1.dpad_right;
        boolean set90RobotLeft = gamepad1.dpad_left;

        if (inc10 || dec10 || set90Field || set90RobotRight || set90RobotLeft)
        {
            if (buttonsEnabled)
            {
                if (inc10)
                    target.moveTargetToRight(10.0);
                if (dec10)
                    target.moveTargetToLeft(10.0);

                if (set90Field)
                    target = TargetDirection.makeTargetAtFieldPosition(90.0);
                if (set90RobotRight)
                    target = TargetDirection.makeTargetToRobotsRight(90.0);
                if (set90RobotLeft)
                    target = TargetDirection.makeTargetToRobotsLeft(90.0);

                buttonsEnabled = false;
            }
        }
        else
        {
            buttonsEnabled = true;
        }

        double distance = target.calculateDistanceFromTarget();
        telemetry.addData("Heading", TargetDirection.getHeading());
        telemetry.addData("Distance From Target", distance);
        telemetry.addData("Power to Motor", turnController.update(distance));
        telemetry.update();
    }

    private void setupImu() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuParameters.temperatureUnit = BNO055IMU.TempUnit.FARENHEIT;

        imu.initialize(imuParameters);
    }
}
