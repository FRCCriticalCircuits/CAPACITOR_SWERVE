package frc.robot.sim.devices;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
public class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFX _kraken;
    private final double _gearRatio;

    private final DCMotorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     *
     * @param falcon
     *                        The TalonFX device
     * @param gearRatio
     *                        The gear ratio from the TalonFX to the mechanism
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX kraken, final double gearRatio, final double rotorInertia) {
        this._kraken = kraken;
        this._gearRatio = gearRatio;
        this._motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, rotorInertia);
    }

    /**
     * Runs the simulation profile.
     *
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        // DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_kraken.getSimState().getMotorVoltage());

        _motorSim.update(getPeriod());

        // SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getAngularPositionRotations() * _gearRatio;
        final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec()) * _gearRatio;

        _kraken.getSimState().setRawRotorPosition(position_rot);
        _kraken.getSimState().setRotorVelocity(velocity_rps);

        _kraken.getSimState().setSupplyVoltage(12 - _kraken.getSimState().getSupplyCurrent() * kMotorResistance);
    }
}