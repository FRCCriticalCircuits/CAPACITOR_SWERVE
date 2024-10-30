package frc.robot.subsystems.swerve.swerveHAL;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.PHYSICAL_CONSTANTS;
import frc.robot.Constants.TUNED_CONSTANTS;
import frc.robot.utils.Conversions.WheelConversions;

public class SwerveModule{
    private TalonFX driveMotor;
    private TalonFXConfiguration driveConfig;

    private CANSparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private PIDController turnController;

    private CANcoder canCoder;

    // drive motor velocity control && FeedForward
    private DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private VelocityVoltage driveVelocity = new VelocityVoltage(0);
    
    private final SimpleMotorFeedforward driveFeedForward;

    /**
     * Create a Swerve Module Object
     * @param driveMotorID : can ID of drive motor
     * @param turningMotorID : can ID of turn motor
     * @param canCoderID : can ID of CANCoder
     * @param CANCoderOffset : CANCoder offset
     * @param turnMotorInverted : turn motor direction
     * @param driveMotorReversed : drive motor direction
     * @param canCoderReversed : CANCoder direction
     */
    public SwerveModule(int driveMotorID, int turningMotorID, int canCoderID, double CANCoderOffset, boolean turnMotorInverted, int driveMotorReversed, Boolean canCoderReversed){
        // Create Motors
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveConfig = new TalonFXConfiguration();
        turnMotor.restoreFactoryDefaults();
        
        // Current Limiting
        turnMotor.enableVoltageCompensation(PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);
        turnMotor.setSmartCurrentLimit(PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.TURN_CURRENT_LIMIT);

        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 0;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0;

        // Neutral Modes
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotor.setIdleMode(IdleMode.kBrake);

        // isInverted
        driveConfig.MotorOutput.Inverted = InvertedValue.valueOf(driveMotorReversed);
        turnMotor.setInverted(turnMotorInverted);

        // CANCoder
        canCoder = new CANcoder(canCoderID, "");

        canCoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(
                    canCoderReversed ?
                        SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive
                ).withMagnetOffset(CANCoderOffset)
        );

        // Turn Feedback Controls
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor((1.0 / PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.TURN_GEAR_RATIO) * Math.PI * 2.0);
        turnEncoder.setVelocityConversionFactor(((1.0 / PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.TURN_GEAR_RATIO) * Math.PI * 2.0) / 60.0);
       
        // Drive Feedback Controls
        driveConfig.Feedback.SensorToMechanismRatio = PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.DRIVE_GEAR_RATIO;

        driveConfig.Slot0.kP = TUNED_CONSTANTS.DRIVEBASE.DRIVE_PIDF0_P;
        driveConfig.Slot0.kI = TUNED_CONSTANTS.DRIVEBASE.DRIVE_PIDF0_I;
        driveConfig.Slot0.kD = TUNED_CONSTANTS.DRIVEBASE.DRIVE_PIDF0_D;

        driveConfig.Voltage.PeakForwardVoltage = 8;
        driveConfig.Voltage.PeakReverseVoltage = -8;

        turnController = new PIDController(TUNED_CONSTANTS.DRIVEBASE.TURN_PIDF0_P, TUNED_CONSTANTS.DRIVEBASE.TURN_PIDF0_I, TUNED_CONSTANTS.DRIVEBASE.TURN_PIDF0_D);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =  PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.DRIVE_LOOP_RAMP_RATE;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.DRIVE_LOOP_RAMP_RATE;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        // FeedForward Controls
        driveFeedForward = new SimpleMotorFeedforward(
            TUNED_CONSTANTS.DRIVEBASE.DRIVE_FEED_FORWARD_KS,
            TUNED_CONSTANTS.DRIVEBASE.DRIVE_FEED_FORWARD_KV
            // No need for kA because voltage doesn't control accelaration
            // No need for kG because it's not arm/elevator
        );

        // Apply Settings
        turnMotor.burnFlash();
        driveMotor.getConfigurator().apply(driveConfig);
    }

    /**
     * get the absolute angle of CANCoder
     * @return the angle in Radians
     */
    public double getAbsoluteAngleRad() {
        return canCoder.getAbsolutePosition().getValue() * Math.PI * 2;
    }

    /**
     * get the absolute angle of CANCoder
     * @return the angle in Degrees
     */
    public double getAbsoluteAngleDeg() {
        return Math.toDegrees(getAbsoluteAngleRad());
    }

    /**
     * get the velocity of Drive Motor
     * @return velocity in meters per second
     */
    public double getDriveVelocity() {
        return WheelConversions.RPSToMPS(driveMotor.getVelocity().getValue(), PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the position of Drive Motor
     * @return distance in meters
     */
    public double getDriveDistance() {
        return WheelConversions.rotationsToMeters(driveMotor.getPosition().getValue(),  PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the angle of turn motor
     * @return angle in radians
     */
    public double getTurnAngleRad() {
        return turnEncoder.getPosition();
    }
    
    /**
     * get the angle of turn motor
     * @return angle in degrees
     */
    public double getTurnAngleDeg() {
        return Math.toDegrees(getTurnAngleRad());
    }

    /**
     * get the angle of turn motor
     * @return angle in {@link Rotation2d}
     */
    public Rotation2d getTurnRotation2D() {
        return Rotation2d.fromRadians(getTurnAngleRad());
    }

    /**
     * get the position(distane and angle) of Swerve Module
     * @return position in {@link SwerveModulePosition} 
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            getTurnRotation2D()
        );
    }

    /**
     * get the state(velocity and angle) of Swerve Module
     * @return state in {@link SwerveModulePosition} 
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            getTurnRotation2D()
        );
    }

    /**
     * set the state of Drive Motor
     * @param desiredState the desired {@link SwerveModuleState} 
     * @param isDutyCycle control mode, duty cycle is true, velocity is false
     */
    private void setDrive(SwerveModuleState desiredState, boolean isDutyCycle){
        if(isDutyCycle){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / PHYSICAL_CONSTANTS.DRIVEBASE.MAX_WHEEL_SPEED_METERS;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = WheelConversions.MPSToRPS(desiredState.speedMetersPerSecond, PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    /**
     * set the state of Turn Motor
     * @param desireState the desired {@link SwerveModuleState} 
     */
    private void setTurn(SwerveModuleState desireState){
        turnMotor.set(turnController.calculate(getTurnAngleRad(),  desireState.angle.getRadians()));
    }

    /**
     * set the state of {@link SwerveModule}
     * @param desireState the desired {@link SwerveModuleState} 
     * @param isDutyCycle control mode, duty cycle is true, velocity is false
     */
    public void setState(SwerveModuleState desireState, boolean isDutyCycle){
        /** Stop Module if speed is lower than 1 percent */
        if(Math.abs(desireState.speedMetersPerSecond) < 0.05){
            stopModule();
            return;
        }

        /** Optimization for turn and Cosine compensation for drive speed */
        Rotation2d currnetAngle = getState().angle;
        desireState = SwerveModuleState.optimize(desireState, currnetAngle);
        desireState.speedMetersPerSecond *= desireState.angle.minus(currnetAngle).getCos();

        /** apply optimized state to motors */ 
        setDrive(desireState, isDutyCycle);
        setTurn(desireState);
    }

    /**
     * reset position of encoders to default value
     * @apiNote drive to 0, turn to CANCoder's position
     */
    public void resetEncoders(){
        driveMotor.getConfigurator().setPosition(0.0); // not used in openloop
        turnEncoder.setPosition(getAbsoluteAngleRad());
    }

    /**
     * stop all motors
     */
    public void stopModule(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
