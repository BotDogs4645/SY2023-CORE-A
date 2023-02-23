// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bit;

import java.util.EnumSet;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.bit.tests.GyroSettingsTest;
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class BITManager {
    private Swerve swerve;
    private Vision vision;

    private SendableChooser<Command> testPicker;

    private boolean available = true;
    private ShuffleboardTab testingTab;
    private SimpleWidget messageWidget;
    private SimpleWidget isTestingWidget;

    public BITManager(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    public void initializeTests() {
        this.testingTab = Shuffleboard.getTab("Built in Tests");
        this.messageWidget = testingTab.add("Message", "yes")
            .withWidget(BuiltInWidgets.kTextView);
        this.testPicker = new SendableChooser<>();

        populatePicker();

        testingTab.add(testPicker);

        GenericEntry listener = testingTab.add("Run test", false)
            .getEntry();

        this.isTestingWidget = testingTab.add("Test Active", false);
        
        NetworkTableListener.createListener(listener, EnumSet.of(Kind.kPublish, Kind.kValueRemote),
        (value) -> {
            if (value.valueData.value.getBoolean() && available) {
                // activate
                testPicker.getSelected().schedule();
            } else {
                listener.accept(NetworkTableValue.makeBoolean(false));
            }
        });
    }
    
    public SimpleWidget getIndicatorWidget() {
        return isTestingWidget;
    }

    public GenericEntry getInstructionsNT() {
        return messageWidget.getEntry();
    }

    private void populatePicker() {
        testPicker.addOption("Gyro Settings Test", new GyroSettingsTest(this, swerve));
    }

    public void setCanUse(boolean canUse) {
        available = canUse;
    }
}
