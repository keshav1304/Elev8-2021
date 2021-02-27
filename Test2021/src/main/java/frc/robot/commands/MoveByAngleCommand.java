// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class MoveByAngleCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double setpoint, error, prevError;

  double integral, derivative;

  /** Creates a new MoveByAngleCommand. */
  public MoveByAngleCommand(DriveSubsystem driveSubsystem, double setpoint) {
    this.driveSubsystem = driveSubsystem;
    this.setpoint = setpoint;


    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navx.reset();
    integral = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.error = this.setpoint - (RobotContainer.navx.getYaw());

    integral += this.error;

    derivative  = this.error - prevError;
    prevError = this.error;

    double correction = (this.error * Constants.kPTurn) + (integral * Constants.kITurn) + (derivative * Constants.kDTurn);
    driveSubsystem.moveByAngle(correction);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(this.error) <= (setpoint * 0.01));
  }
}
