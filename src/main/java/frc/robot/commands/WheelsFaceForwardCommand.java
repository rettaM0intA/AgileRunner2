// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WheelsFaceForwardCommand extends CommandBase {

  double fRSetAngle;
  double fLSetAngle;
  double bRSetAngle;
  double bLSetAngle;

  boolean firstRunHappened = false;

  /**
   * Unfinished command that will cause the wheels to face towards the front of the robot.  Used before a match or at the beginning of autonomous mode although before is prefered.
   */
  public WheelsFaceForwardCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.m_chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot.sub_c.driveTeleop(0, 0, 0);

    // fRSetAngle = ((Robot.sub_c.fRAnalogEncoder.get() - 1 - Constants.kChassisAbsoluteZerofR) * 90) / Constants.kChassisSwerveOutputDegreeToNeoRotation;

    // Robot.sub_c.fRrotationMotor.getEncoder().setPosition(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Robot.sub_c.driveTeleop(0, 0, 0);

    // firstRunHappened = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return false;
  }
}
