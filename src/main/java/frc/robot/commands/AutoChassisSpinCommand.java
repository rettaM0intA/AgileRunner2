// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoChassisSpinCommand extends CommandBase {

  double speed = 0;
  double distance = 1;
  double travelDistance = 0;
  boolean isInit = false;
  boolean isFinished = false; // boolean used to end the command. is necessary since the isFinished function
                              // is called immediatly.

  int buffer = 0;

  double goalDegree = 0;
  double currentDegree = 0;
 
  /**
   * 
   * @param m_degree   What direction you want to go in degrees
   * @param m_speed    How fast you want to move in percent
   * 
   *                   Makes the chassis move in a desired direction at a desired
   *                   speed.
   */
  public AutoChassisSpinCommand(double m_goalDegree, double m_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);

    goalDegree = m_goalDegree;
    speed = m_speed * 0.01;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_chassisSubsystem.zeroMotors();
    buffer = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    buffer += 1;

    currentDegree = RobotContainer.m_chassisSubsystem.gyro.getAngle();

    if(goalDegree > 0){
      if(buffer > 18){
        RobotContainer.m_chassisSubsystem.driveAuton(0, 0, -speed);
      }else{
        RobotContainer.m_chassisSubsystem.driveAuton(0, 0, -0.01);
      
      }
     
    } else {
      if(buffer > 18){
        RobotContainer.m_chassisSubsystem.driveAuton(0, 0, speed);
      }else{
        RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0.01);
      }
    }
   


    if (goalDegree != 0) {
      if ((goalDegree > 0 && currentDegree > goalDegree) || (goalDegree < 0 && currentDegree < goalDegree)) {
        RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
        isFinished = true;
      } else {
        isFinished = false;
      }
    } else {
      RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
