
package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  private final PWMVictorSPX leftBack = new PWMVictorSPX(0);
  private final PWMVictorSPX leftFront = new PWMVictorSPX(1);
  private final PWMVictorSPX rightBack = new PWMVictorSPX(2);
  private final PWMVictorSPX rightFront = new PWMVictorSPX(3);

  private final SpeedControllerGroup leftSCG = new SpeedControllerGroup(leftBack, leftFront);
  private final SpeedControllerGroup rightSCG = new SpeedControllerGroup(rightBack, rightFront);

  private final DifferentialDrive drive = new DifferentialDrive(leftSCG, rightSCG);

  private final PWMVictorSPX elevator1 = new PWMVictorSPX(4);
  private final PWMVictorSPX elevator2 = new PWMVictorSPX(5);

  private final SpeedControllerGroup elevatorSCG = new SpeedControllerGroup(elevator1, elevator2);

  private final Joystick joystick = new Joystick(0);

  private final JoystickButton button0 = new JoystickButton(joystick, 0);
  private final JoystickButton button1 = new JoystickButton(joystick, 1);

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    drive.tankDrive(0.5, 0.5);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drive.tankDrive(joystick.getRawAxis(1), joystick.getRawAxis(3));

    leftSCG.set(joystick.getRawAxis(1));
    rightSCG.set(joystick.getRawAxis(3));

    if (button0.get()) {
      elevatorSCG.set(0.7);
    } else {
      elevatorSCG.set(-0.7);
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
