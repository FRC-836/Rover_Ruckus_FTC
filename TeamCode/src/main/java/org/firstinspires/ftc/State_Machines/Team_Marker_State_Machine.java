package org.firstinspires.ftc.State_Machines;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;

public class Team_Marker_State_Machine implements Runnable {
    private Servo teamMarkerServo;
    private State currentState;
    private AtomicBoolean opModeIsActive = new AtomicBoolean(true);
    private ElapsedTime timer = new ElapsedTime();

    private Semaphore lock = new Semaphore(1);

    enum State {
        UP,
        DOWN
    }

    public Team_Marker_State_Machine(HardwareMap hardwareMap) {
        teamMarkerServo = hardwareMap.get(Servo.class, "tms");
        teamMarkerServo.setDirection(Servo.Direction.FORWARD);

        teamMarkerServo.setPosition(1.0);
        currentState = State.UP;
    }

    @Override
    public void run() {
        while (opModeIsActive.get()) {
            try {
                lock.acquire();
                if (currentState == State.DOWN){
                    teamMarkerServo.setPosition(0.00);
                    if (timer.seconds() > 1.1){
                        currentState = State.UP;
                    }
                } else if (currentState == State.UP){
                    teamMarkerServo.setPosition(1.00);
                }
                lock.release();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            sleep(15);
        }
    }

    public void stop (){
        opModeIsActive.set(false);
    }

    public void dropTeamMarker() {
        while (!lock.tryAcquire()) // TODO: Determine if the '!' should be here
        {
            sleep(0);
        }
        currentState = State.DOWN;
        timer.reset();
        lock.release();
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
