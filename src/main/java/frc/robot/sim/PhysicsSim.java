package frc.robot.sim;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.sim.devices.TalonFXSimProfile;

/**
 * Manages physics simulation for CTRE products.
 */
public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();
    private ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Adds a TalonFX controller to the simulator.
     * 
     * @param kraken
     *        The TalonFX device
     * @param gearRatio
     *        The gear reduction of the TalonFX
     * @param rotorInertia
     *        Rotational Inertia of the mechanism at the rotor
     */
    public void addTalonFX(TalonFX kraken, double gearRatio, final double rotorInertia) {
        if (kraken != null) {
            TalonFXSimProfile simkraken = new TalonFXSimProfile(kraken, gearRatio, rotorInertia);
            _simProfiles.add(simkraken);
        }
    }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return sim;
    }
}