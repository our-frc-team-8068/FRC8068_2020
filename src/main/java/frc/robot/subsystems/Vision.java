/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogitechGamePad;
import frc.robot.LogitechJoystick;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private ShuffleboardTab driverInformationCenterTab = Shuffleboard.getTab("DriverInformationCenter"); 
  private NetworkTableEntry cameraSelection;
  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;

  private UsbCamera shootCamera;
  private UsbCamera controlPanelCamera;
  private VideoSink server;
  private VideoMode shootVideoMode = new VideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);

  private boolean switchCameras = false;

  public Vision(Joystick driverJoystick, Joystick operatorJoystick) {
    this.driverJoystick = driverJoystick;
    this.operatorJoystick = operatorJoystick;
    shootCamera = CameraServer.getInstance().startAutomaticCapture("Shooter Camera", 0);
    //shootCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 176, 144, 30);
    shootCamera.setVideoMode(shootVideoMode);
    //shootCamera.setResolution(320, 240);
    //shootCamera.setFPS(15);
    controlPanelCamera = CameraServer.getInstance().startAutomaticCapture("Control Panel Camera", 1);
    //controlPanelCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
    controlPanelCamera.setResolution(320, 240);
    controlPanelCamera.setFPS(15);

    server = CameraServer.getInstance().getServer();
    
    cameraSelection = NetworkTableInstance.getDefault().getTable("CameraPublisher").getEntry(
      "USB Camera 1"); 

    driverInformationCenterTab.add(server.getSource());    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (operatorJoystick.getRawButtonPressed(LogitechJoystick.BUTTON_3)) 
    {
      switchCameras = !switchCameras;
    }
    if (switchCameras)
    {
      server.setSource(controlPanelCamera); 
    }
    else
    {
      server.setSource(shootCamera); 
    }
  }
}
