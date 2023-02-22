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
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class BITManager {
    private Swerve swerve;
    private Vision vision;

    private ShuffleboardTab testingTab;
    private SimpleWidget messageWidget;

    public BITManager(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    public void initializeTests() {
        this.testingTab = Shuffleboard.getTab("Built in Tests");
        this.messageWidget = testingTab.add("Message", "yes")
            .withWidget(BuiltInWidgets.kTextView);
        SendableChooser<Command> testPicker = new SendableChooser<>();
        
        testingTab.add(testPicker);

        GenericEntry listener = testingTab.add("Run test", false)
            .getEntry();
        
        NetworkTableListener.createListener(listener, EnumSet.of(Kind.kPublish),
        (value) -> {
            if (value.valueData.value.getBoolean()) {
                // activate
                testPicker.getSelected().schedule();
                listener.accept(NetworkTableValue.makeBoolean(false));
            }
        });
    }
}
