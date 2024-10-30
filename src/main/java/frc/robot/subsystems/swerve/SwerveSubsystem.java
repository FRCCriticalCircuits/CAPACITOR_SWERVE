package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DEVICE_IDs;
import frc.robot.Constants.PHYSICAL_CONSTANTS;
import frc.robot.subsystems.swerve.swerveHAL.SwerveModule;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.Math.AdvancedPose2D;

public class SwerveSubsystem extends SubsystemBase{
    private static SwerveSubsystem instance;

    AdvancedPose2D initPose = new AdvancedPose2D(1.32, 5.55, Rotation2d.fromDegrees(0));
    
    // Hardwares
    private SwerveModule frontLeft, frontRight, rearLeft, rearRight;
    private AHRS gyro;

    // PoseEstimation
    private SwerveDrivePoseEstimator poseEstimator;

    // telemetry
    private Field2d estimateField = new Field2d();

    private final StructArrayPublisher<SwerveModuleState> currentSwerveStatePublisher;
    private final StructArrayPublisher<SwerveModuleState> desireSwerveStatePublisher;

    private SwerveSubsystem(){
        // Swerve Modules
        frontLeft = new SwerveModule(
            DEVICE_IDs.DRIVEBASE.FRONT_LEFT_DRIVE_ID, 
            DEVICE_IDs.DRIVEBASE.FRONT_LEFT_TURN_ID, 
            DEVICE_IDs.DRIVEBASE.FRONT_LEFT_CANCODER_ID, 
            PHYSICAL_CONSTANTS.DRIVEBASE.CANCODER.FRONT_LEFT_OFFSET, 
            true,
            0,
            true
        );

        frontRight = new SwerveModule(
            DEVICE_IDs.DRIVEBASE.FRONT_RIGHT_DRIVE_ID, 
            DEVICE_IDs.DRIVEBASE.FRONT_RIGHT_TURN_ID, 
            DEVICE_IDs.DRIVEBASE.FRONT_RIGHT_CANCODER_ID, 
            PHYSICAL_CONSTANTS.DRIVEBASE.CANCODER.FRONT_RIGHT_OFFSET, 
            true,
            1,
            true
        );

        rearLeft = new SwerveModule(
            DEVICE_IDs.DRIVEBASE.REAR_LEFT_DRIVE_ID, 
            DEVICE_IDs.DRIVEBASE.REAR_LEFT_TURN_ID, 
            DEVICE_IDs.DRIVEBASE.REAR_LEFT_CANCODER_ID, 
            PHYSICAL_CONSTANTS.DRIVEBASE.CANCODER.REAR_LEFT_OFFSET, 
            true,
            0, 
            true
        );

        rearRight = new SwerveModule(
            DEVICE_IDs.DRIVEBASE.REAR_RIGHT_DRIVE_ID, 
            DEVICE_IDs.DRIVEBASE.REAR_RIGHT_TURN_ID, 
            DEVICE_IDs.DRIVEBASE.REAR_RIGHT_CANCODER_ID, 
            PHYSICAL_CONSTANTS.DRIVEBASE.CANCODER.REAR_RIGHT_OFFSET, 
            true,
            1,
            true
        );

        // gyro
        gyro = new AHRS(DEVICE_IDs.DRIVEBASE.GYRO_PORT);
        gyro.setAngleAdjustment(0);

        // reset gyro & encoders
        new Thread(
            () -> {
                try {
                    Thread.sleep(10);
                    frontLeft.resetEncoders();
                    Thread.sleep(10);
                    frontRight.resetEncoders();
                    Thread.sleep(10);
                    rearLeft.resetEncoders();
                    Thread.sleep(10);
                    rearRight.resetEncoders();
                    Thread.sleep(1000);
                    gyro.zeroYaw();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        ).start();

        // Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.PHYSICAL_CONSTANTS.DRIVEBASE.KINEMATICS, 
            getGyroRotation2D(),
            getSwerveModulePositions(),
            DriveStationIO.getInstance().isBlue() ? initPose : initPose.flip(0, 0)
        );

        // Pathplanner
        AutoBuilder.configureHolonomic(
            this::getPoseEstimate, 
            this::resetPoseEstimate, 
            this::getChassisSpeeds,
            this::setModuleStates, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(0),
                new PIDConstants(0),
                PHYSICAL_CONSTANTS.DRIVEBASE.MAX_WHEEL_SPEED_METERS,
                PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.TRACK_RADIUS_METERS,
                new ReplanningConfig(true, true),
                Constants.LOOP_TIME_S
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this
        );

        // Telemetry
        currentSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/CurrentState", SwerveModuleState.struct).publish();
        desireSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/DesiredState", SwerveModuleState.struct).publish();
    }

    public static SwerveSubsystem getInstance(){
        if(instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    /**
     * Get the heading of gyroscope
     * @return the CCW(Counter ClockWise Positive) angle in degrees
     */
    public double getHeading() {
        return gyro.getYaw() * PHYSICAL_CONSTANTS.GYRO_REVERSE_FACTOR;
    }

    /**
     * Get the heading of gyroscope
     * @return the heading in {@link Rotation2D}
     */
    public Rotation2d getGyroRotation2D() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * reset the heading(yaw) of gyroscope to zero
     */
    public void resetYaw(){
        gyro.zeroYaw();
    }

    /**
     * get the four Modules' Position
     * @return the position in {@link SwerveModulePosition} 
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition()
        }; 
    }

    /**
     * get the four Modules' State
     * @return the position in {@link SwerveModuleState} 
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        };
    }

    /**
     * get current chassis speed
     * @return {@link ChassisSpeeds} speed
     */
    public ChassisSpeeds getChassisSpeeds() {
        return PHYSICAL_CONSTANTS.DRIVEBASE.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * set the state of four modules with a {@link SwerveModuleState} array
     * @param states the desired {@link SwerveModuleState} array
     * @param isOpenLoop true if using duty cycle
     */
    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop){
        // normalize wheelspeed to make it smaller than the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, PHYSICAL_CONSTANTS.DRIVEBASE.MAX_WHEEL_SPEED_METERS);

        // apply speeds to each Swerve Module
        frontLeft.setState(states[0], isOpenLoop);
        frontRight.setState(states[1], isOpenLoop);
        rearLeft.setState(states[2], isOpenLoop);
        rearRight.setState(states[3], isOpenLoop);

        // telemetry
        desireSwerveStatePublisher.set(states);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param states the desired {@link ChassisSpeeds} speed
     * @param isOpenLoop true if using duty cycle
     */
    public void setModuleStates(ChassisSpeeds speeds, boolean isOpenLoop){
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState states[] = PHYSICAL_CONSTANTS.DRIVEBASE.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states, isOpenLoop);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param states the desired {@link ChassisSpeeds} speed
     * @apiNote control mode is <b>dutyCycle</b> by default
     */
    public void setModuleStates(ChassisSpeeds speeds){
        setModuleStates(speeds, true);
    }

    /**
     * get the position of robot from the {@link SwerveDrivePoseEstimator}
     * @return position in {@link Pose2d}
     */
    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * reset the position of the {@link SwerveDrivePoseEstimator} 
     * @param pose new position in {@link Pose2d}
     */
    public void resetPoseEstimate(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation2D(), getSwerveModulePositions(), pose);
    }

    /**
     * update the {@link SwerveDrivePoseEstimator}
     */
    public void updatePoseEstimators() {
        poseEstimator.update(getGyroRotation2D(), getSwerveModulePositions());
    }

    @Override
    public void periodic(){
        // Pose Estimator
        updatePoseEstimators();

        // Telemetry
        estimateField.setRobotPose(getPoseEstimate());
        currentSwerveStatePublisher.set(getSwerveModuleStates());
    }
}
