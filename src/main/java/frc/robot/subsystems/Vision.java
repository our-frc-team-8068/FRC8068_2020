/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogitechGamePad;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private ShuffleboardTab driverInformationCenterTab = Shuffleboard.getTab("DriverInformationCenter"); 
  private NetworkTableEntry cameraSelection;
  private final Joystick driverJoystick;

  private UsbCamera shootCamera;
  private UsbCamera controlPanelCamera;
  private VideoSink server;

  public Vision(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;
    shootCamera = CameraServer.getInstance().startAutomaticCapture(0);
    shootCamera.setResolution(320, 240);
    shootCamera.setFPS(15);
    
    controlPanelCamera = CameraServer.getInstance().startAutomaticCapture(1);
    controlPanelCamera.setResolution(320, 240);
    controlPanelCamera.setFPS(15);
    
    server = CameraServer.getInstance().getServer();
    //server.setSource(shootCamera);
    cameraSelection = NetworkTableInstance.getDefault().getTable("CameraPublisher").getEntry(
      "USB Camera 1"); 

    driverInformationCenterTab.add(server.getSource());    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (driverJoystick.getPOV() == 0) 
    {
      server.setSource(shootCamera); 
    }
    else if (driverJoystick.getPOV() == 90)
    {
      server.setSource(controlPanelCamera); 
       
    }
  }
}
