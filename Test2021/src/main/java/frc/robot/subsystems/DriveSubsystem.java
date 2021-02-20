// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double acceleration;
  private double velocity;
  private double displacement; 
  private double previousTime;
  private Timer timer;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    rightSide = new SpeedControllerGroup(FR, BR);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 
    leftSide = new SpeedControllerGroup(FL, BL);

    resetIntegrals();
    RobotContainer.navx.reset();
    timer.reset();
    coastMode();
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);

    timer.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDistance();

    SmartDashboard.putNumber("Displacement", this.displacement);
    SmartDashboard.putNumber("Velocity", this.velocity);
    SmartDashboard.putNumber("Acceleration", this.acceleration);

  }

  public void arcadeInbuilt(double y, double z) {
    FR.setInverted(false);
    BR.setInverted(false);
    driveTrain.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  public void brakeMode() {
    FL.setNeutralMode(NeutralMode.Brake);
    FR.setNeutralMode(NeutralMode.Brake);
    BL.setNeutralMode(NeutralMode.Brake);
    BR.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    FL.setNeutralMode(NeutralMode.Coast);
    FR.setNeutralMode(NeutralMode.Coast);
    BL.setNeutralMode(NeutralMode.Coast);
    BR.setNeutralMode(NeutralMode.Coast);
  }

  public void drive(double l, double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void resetIntegrals() {
    this.velocity = 0.0d;
    this.displacement = 0.0d;
  }

  public void updateDistance() {

    double deltaTime = timer.get() - this.previousTime;
    this.previousTime += deltaTime;

    SmartDashboard.putNumber("Timer", previousTime);

    this.acceleration = RobotContainer.navx.getWorldLinearAccelX() * Constants.G;
    this.velocity += this.acceleration * deltaTime;
    this.displacement += this.velocity * deltaTime;
  }


}
