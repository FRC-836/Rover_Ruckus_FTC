package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Heading Test")
public class HeadingTest extends OpMode {

    Heading heading;
    private BNO055IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        setupImu();

        Heading.setImu(imu);
        Heading.setFieldOffset(-Heading.getAbsoluteHeading());

        heading = Heading.createFieldHeading(0.0f);
    }

    @Override
    public void loop() {

        if (gamepad1.x)
            heading.setRelativeOffset(0.0f);

        if (gamepad1.y)
            heading.setRelativeOffset(90.0f);

        if (gamepad1.a)
            heading = Heading.createRelativeHeading(90.0f);

        if (gamepad1.b)
            heading = Heading.createRelativeHeading(0.0f);

        telemetry.addData("Absolute", Heading.getAbsoluteHeading());
        telemetry.addData("Field", Heading.getFieldHeading());
        telemetry.addData("Relative", heading.getRelativeHeading());
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

        imu.initialize(imuParameters);
    }
}
