// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ABPathCommand extends SequentialCommandGroup {
  /** Creates a new ABPathCommand. */
  public ABPathCommand(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    //addCommands(new MoveByDistanceCommand(driveSubsystem, 3 * Constants.FIELD));
    addCommands(new MoveByDistanceCommand(driveSubsystem,  2.286));
    //addCommands(new MoveByAngleCommand(driveSubsystem, Math.atan(2/3)));
    addCommands(new MoveByAngleCommand(driveSubsystem, 42));
    //addCommands(new MoveByDistanceCommand(driveSubsystem, Math.sqrt(13) * Constants.FIELD));
    addCommands(new MoveByDistanceCommand(driveSubsystem, 3));
    //addCommands(new MoveByAngleCommand(driveSubsystem, -1 * Math.atan(2/3)));
    //addCommands(new MoveByAngleCommand(driveSubsystem, -1 * Math.atan(1/3)));
    addCommands(new MoveByAngleCommand(driveSubsystem, -94));
    //addCommands(new MoveByDistanceCommand(driveSubsystem, Math.sqrt(18) * Constants.FIELD));
    addCommands(new MoveByDistanceCommand(driveSubsystem, 3.4));
    //addCommands(new MoveByAngleCommand(driveSubsystem, Math.atan(2/3)));
    //addCommands(new MoveByAngleCommand(driveSubsystem, Math.atan(1/3)));
    //addCommands(new MoveByAngleCommand(driveSubsystem, Math.atan(2)));
    addCommands(new MoveByAngleCommand(driveSubsystem, 115));
    addCommands(new MoveByDistanceCommand(driveSubsystem, 2 * Constants.FIELD));
    addCommands(new MoveByAngleCommand(driveSubsystem, -70));
    addCommands(new MoveByDistanceCommand(driveSubsystem, 1 * Constants.FIELD));

  }
}
