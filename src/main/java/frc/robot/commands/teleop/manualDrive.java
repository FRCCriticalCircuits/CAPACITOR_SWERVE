package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PHYSICAL_CONSTANTS;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class manualDrive extends Command{
    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpdFunc, ySpdFunc, tSpdFunc, scaleFunc0, scaleFunc1;

    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public manualDrive(
        Supplier<Double> xSpdFunc,
        Supplier<Double> ySpdFunc,
        Supplier<Double> tSpdFunc,
        Supplier<Double> scaleFunc0,
        Supplier<Double> scaleFunc1
    ){
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.tSpdFunc = tSpdFunc;
        this.scaleFunc0 = scaleFunc0;
        this.scaleFunc1 = scaleFunc1;

        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.thetaLimiter = new SlewRateLimiter(3);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        // get reading from Supplier
        double xSpeed = xSpdFunc.get();
        double ySpeed = ySpdFunc.get();
        double turningSpeed = tSpdFunc.get();
        double scale0 = scaleFunc0.get();
        double scale1 = scaleFunc1.get();

        // Apply Deadband to Controller Inputs
        xSpeed = Math.abs(xSpeed) > 0.15 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.15 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.15 ? turningSpeed : 0.0;

        // Normalization
        if(scale0 < 0.3) scale0 = 0.3;
        if(scale1 < 0.3) scale1 = 0.3;

        xSpeed *= (scale0 + scale1) * 0.5;
        ySpeed *= (scale0 + scale1) * 0.5;

        // Apply Limiter
        xSpeed = xLimiter.calculate(xSpeed) * PHYSICAL_CONSTANTS.DRIVEBASE.MAX_WHEEL_SPEED_METERS;
        ySpeed = yLimiter.calculate(ySpeed) * PHYSICAL_CONSTANTS.DRIVEBASE.MAX_WHEEL_SPEED_METERS;
        turningSpeed = thetaLimiter.calculate(turningSpeed) * PHYSICAL_CONSTANTS.DRIVEBASE.MAX_ANGULAR_SPEED_RAD;

        // Apply speeds to swervemodule
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getGyroRotation2D());
        //ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getGyroRotation2D());
        swerveSubsystem.setModuleStates(chassisSpeeds, true);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
