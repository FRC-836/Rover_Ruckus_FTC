package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Arm PID Testing", group = "A")
public class Arm_PID_Test extends LinearOpMode{
    private DcMotor armMotor;
    protected PID_Controller moveArmPid = new PID_Controller(0.00042,0.00028,0.0 );
    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        moveArmPid.setSetpoint(armMotor.getCurrentPosition() + 1800);
        while(opModeIsActive()) {
            armMotor.setPower(moveArmPid.update(armMotor.getCurrentPosition()));
            moveArmPid.displayCurrentPID(telemetry);
            telemetry.addData("Goal: ", moveArmPid.getSetpoint());
            telemetry.addData("Arm Encoder",armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

/* moving 1800 encoder counts (positive)
p : 0.00042
i : 0.00028
d :
 */