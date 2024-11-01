package frc.robot.sim;

import com.ctre.phoenix6.Utils;

/**
 * Holds information about a simulated device.
 */
public class SimProfile {
    private double _lastTime;
    private boolean _running = false;

    /**
     * Runs the simulation profile.
     * Implemented by device-specific profiles.
     */
    public void run() {}

    /**
     * Returns the time since last call, in seconds.
     */
    protected double getPeriod() {
        // set the start time if not yet running
        if (!_running) {
            _lastTime = Utils.getCurrentTimeSeconds();
            _running = true;
        }

        double now = Utils.getCurrentTimeSeconds();
        final double period = now - _lastTime;
        _lastTime = now;

        return period;
    }
}