// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final SpeedControllerGroup rightSide;

  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  private final SpeedControllerGroup leftSide;

  private final DifferentialDrive driveTrain;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    rightSide = new SpeedControllerGroup(FR, BR);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 
    leftSide = new SpeedControllerGroup(FL, BL);
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(final double l, final double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void arcadeInbuilt(final double y, final double z) {
    FR.setInverted(false);
    BR.setInverted(false);

    driveTrain.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  public void moveByAngle(double correction) {
    if (Math.abs(correction) < 0.13) correction = Math.signum(correction) * 0.1;
    if (Math.abs(correction) > 0.6) correction = Math.signum(correction) * 0.6 ;
    drive(correction * Constants.maxSpeed, -1 * correction * Constants.maxSpeed);
  }
}

