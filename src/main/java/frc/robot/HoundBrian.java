package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.LEDs.LEDState;

@LoggedObject
public class HoundBrian {
    @Log
    private final DigitalInput drivetrainButton = new DigitalInput(0);
    @Log
    private final DigitalInput shooterHoodButton = new DigitalInput(1);
    @Log
    private final DigitalInput intakeButton = new DigitalInput(2);
    @Log
    private final DigitalInput switch1 = new DigitalInput(5);
    @Log
    private final DigitalInput switch2 = new DigitalInput(6);

    public HoundBrian(Drivetrain drivetrain, ShooterHood shooterHood, Intake intake, LEDs leds) {

        new Trigger(drivetrainButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true)
                        .alongWith(leds.requestStateCommand(LEDState.HOUNDBRIAN_CLICK).withTimeout(0.5)));

        new Trigger(shooterHoodButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(shooterHood.resetPositionCommand().ignoringDisable(true)
                        .alongWith(leds.requestStateCommand(LEDState.HOUNDBRIAN_CLICK).withTimeout(0.5)));
        new Trigger(intakeButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(intake.resetPositionCommand().ignoringDisable(true)
                        .alongWith(leds.requestStateCommand(LEDState.HOUNDBRIAN_CLICK).withTimeout(0.5)));

        // new Trigger(switch1::get)
        // .whileTrue(leds.requestStateCommand(LEDState.DEMO_RED).ignoringDisable(true));
        // new Trigger(switch2::get)
        // .whileTrue(leds.requestStateCommand(LEDState.DEMO_GOLD).ignoringDisable(true));
    }
}
