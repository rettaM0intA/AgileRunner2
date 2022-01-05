// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoChassisMovePid extends CommandBase {

  boolean isFinished;

  double goalRadian;
  double speed;
  
  double fwd;
  double strafe;

  double distanceFrontRight;
  double distanceFrontLeft;
  double distanceBackRight;
  double distanceBackLeft;

  /** Creates a new AutoChasssisMovePid.
   * 
   * @param m_degree What direction you want to go in degrees
   * @param m_speed How fast you want to move in percent
   * @param m_distanceFrontLeft How far the robot will travel in inches
   * @param m_distanceFrontRight How far the robot will travel in inches
   * @param m_distanceBackLeft How far the robot will travel in inches
   * @param m_distanceBackRight How far the robot will travel in inches
   * 
   */
  public AutoChassisMovePid(double m_degree, double m_speed, double m_distanceFrontLeft, double m_distanceFrontRight, double m_distanceBackLeft, double m_distanceBackRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);

    goalRadian = ((m_degree + 90) * Math.PI / 180); //The math requires radians, so translate degree input to radians
    speed = m_speed;

    
    fwd = (Math.sin(goalRadian) / 100) * speed;
    strafe = (Math.cos(goalRadian) / 100) * speed;

    distanceFrontLeft = -m_distanceFrontLeft * Constants.kChassisEstimatedRotationsToInches;
    distanceFrontRight= m_distanceFrontRight * Constants.kChassisEstimatedRotationsToInches;
    distanceBackLeft = -m_distanceBackLeft * Constants.kChassisEstimatedRotationsToInches;
    distanceBackRight = m_distanceBackRight * Constants.kChassisEstimatedRotationsToInches;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_chassisSubsystem.zeroMotors();
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.m_chassisSubsystem.driveToPoint(fwd, strafe, 0, distanceFrontLeft, distanceFrontRight, distanceBackLeft, distanceBackRight);
    //RobotContainer.m_chassisSubsystem.driveToPoint(0, 0, 0, 100000, -100000, 100000, -100000);

    if(RobotContainer.m_chassisSubsystem.checkPIDlocation()){
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      RobotContainer.m_chassisSubsystem.resetGyro();
      RobotContainer.m_chassisSubsystem.zeroMotors();
      RobotContainer.m_chassisSubsystem.disablePids();

      return true;
    }
    return false;
  }
}
