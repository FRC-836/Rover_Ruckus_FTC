package org.firstinspires.ftc.teamcode;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Drive_Runnable implements Runnable {

    private AtomicInteger counter = new AtomicInteger(0);
    private AtomicBoolean isShutdown = new AtomicBoolean(false);

    @Override
    public void run() {
        while (!isShutdown.get()) {
            counter.incrementAndGet();
        }
    }

    public void shutdown() {
        isShutdown.set(true);
    }

    public int getDriveCounter() {
        return counter.get();
    }
}
