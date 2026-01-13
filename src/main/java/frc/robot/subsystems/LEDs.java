package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs.LEDSection;

import static frc.robot.Constants.LEDs.*;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

/**
 * The LED subsystem, which controls the state of the LEDs by superimposing
 * requested LED states and continuously updates the LED's buffer. Other classes
 * can request specific LED states to be active, and they will be applied in
 * priority order.
 * 
 * @author dr
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    private AddressableLED leds = new AddressableLED(6);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    /** Notifier thread for displaying when the robot code is initializing. */
    private final Notifier loadingNotifier;

    private List<LEDState> activeLEDStates = new ArrayList<>();

    public enum LEDState {
        HOUNDBRIAN_CLICK(solid(Color.kYellow, LEDSection.ALL)),

        DRIVETRAIN_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.ALL)),
        INITIALIZATION_BLACK_BACKGROUND(solid(Color.kBlack, LEDSection.ALL)),

        INITIALIZED_CONFIRM(breathe(Color.kGreen, 2, 0, 255, LEDSection.ALL)),

        READY_TO_SCORE(solid(Color.kGreen, LEDSection.ALL)),

        READY_FOR_CORAL(solid(Color.kOrange, LEDSection.ALL)),

        GOLD_WAVE(wave(new Color("#FBBF05"), 25, 10, 50, 150, LEDSection.ALL));

        private List<Consumer<AddressableLEDBuffer>> bufferConsumers;

        @SafeVarargs
        private LEDState(Consumer<AddressableLEDBuffer>... bufferConsumer) {
            this.bufferConsumers = Arrays.asList(bufferConsumer);
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        leds.setLength(LENGTH);
        leds.setData(buffer);
        leds.start();

        loadingNotifier = new Notifier(() -> updateLoadingLEDs());
        loadingNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    /**
     * Creates a command that requests a specific LED state to be active. When
     * the command is cancelled, the state will no longer be active.
     * 
     * @param state the state to request
     * @return the command
     */
    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> activeLEDStates.add(state)).ignoringDisable(true).withName("leds.requestState");
    }

    /**
     * Creates a command that updates the LED buffer with the contents of the
     * current LED states.
     * 
     * @return the update buffer command
     */
    public Command updateBufferCommand() {
        return run(() -> updateLEDs()).ignoringDisable(true).withName("leds.updateBuffer");
    }

    /**
     * Updates the LED buffer based on active states.
     */
    private void updateLEDs() {
        loadingNotifier.stop();
        clear();

        if (DriverStation.isDisabled()) {
            activeLEDStates.addAll(DEFAULT_STATES);
        }

        activeLEDStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
        activeLEDStates.forEach(state -> {
            state.bufferConsumers.forEach(consumer -> consumer.accept(buffer));

            // if (state == LEDState.ALGAE_DETECTOR) {

            // }
        });

        leds.setData(buffer);
        activeLEDStates.clear();
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }

    /**
     * Updates the LEDs to indicate loading state.
     */
    private void updateLoadingLEDs() {
        synchronized (this) {
            breathe(Color.kWhite, 3, 0, 255, LEDSection.ALL).accept(buffer);
            leds.setData(buffer);
        }
    }
}
