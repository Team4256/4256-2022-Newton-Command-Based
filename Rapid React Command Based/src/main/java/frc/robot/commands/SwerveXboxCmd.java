// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.*;
public class SwerveXboxCmd extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveXboxCmd(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Parameters.TELEOP_MAX_ACCELERATION);
    this.yLimiter = new SlewRateLimiter(Parameters.TELEOP_MAX_ACCELERATION);
    this.turningLimiter = new SlewRateLimiter(Parameters.TELEOP_MAX_ANGULAR_ACCELERATION);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
     // Get Xbox inputs
     double xSpeed = xSpdFunction.get();
     double ySpeed = ySpdFunction.get();
     double turningSpeed = turningSpdFunction.get();

     // 3. Make the driving smoother
     xSpeed = xLimiter.calculate(xSpeed) * Parameters.TELEOP_SPEED_LIMIT_MPS;
     ySpeed = yLimiter.calculate(ySpeed) * Parameters.TELEOP_SPEED_LIMIT_MPS;
     turningSpeed = turningLimiter.calculate(turningSpeed)
             * Parameters.TELEOP_ANGULAR_SPEED_LIMIT_RADIANS_PER_SECOND;

     // 4. Construct desired chassis speeds
     ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
     // 5. Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = Parameters.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

     // 6. Output each module states to wheels
     swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
