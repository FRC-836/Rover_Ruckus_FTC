package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Arm PID Testing", group = "A")
public class Arm_PID_Test extends LinearOpMode{
    private DcMotor armMotor;

    private boolean repeated = false;

    protected PID_Controller moveArmPid = new PID_Controller(0.00165,0.000,0.000375 );
    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        moveArmPid.setSetpoint(armMotor.getCurrentPosition() + 1800);
        moveArmPid.resetPID();

        int resetPoint = armMotor.getCurrentPosition() + 1100;

        while(opModeIsActive()) {
            armMotor.setPower(moveArmPid.update(armMotor.getCurrentPosition()));
            if (armMotor.getCurrentPosition() >= resetPoint && !repeated) {
                repeated = true;
                moveArmPid.resetPID();
            }
            moveArmPid.displayCurrentPID(telemetry);
            telemetry.addData("Goal: ", moveArmPid.getSetpoint());
            telemetry.addData("Arm Encoder",armMotor.getCurrentPosition());
            telemetry.addData("Int stopped", repeated);
            telemetry.update();
        }
    }
}

/* moving 1800 encoder counts (positive)
p : 0.003
i : 0.00028
d : 0.00042

Integral for weight: 0.00017
 */