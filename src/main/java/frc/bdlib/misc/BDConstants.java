// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bdlib.misc;

import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class BDConstants {
    public static final class JoystickConstants {
        // add ids in this order
        public static enum JoystickButtonID {
            NONE,
            kA,
            kB,
            kX,
            kY,
            kLeftBumper,
            kRightBumper,
            kLeftStick,
            kRightStick,
            kBack,
            kStart
        }

        public static enum JoystickAxisID {
            kLeftX,
            kLeftY,
            kRightX,
            kRightY,
            kLeftTrigger,
            kRightTrigger
        }

        public static enum JoystickVariant {
            XBOX("Controller (Xbox One For Windows)", new int[] {0,1,2,3,4,5,6,9,10,7,8}, new int[] {0,1,4,5,2,3}, true),
            ;

            public static Optional<JoystickVariant> findJoy(String idf) {
                for (JoystickVariant variant : JoystickVariant.values()) {
                    if (variant.getIdf().equals(idf)) {
                        return Optional.of(variant);
                    }
                }
                return Optional.empty();
            }
            
            final Map<JoystickButtonID, Integer> buttonBinds = new EnumMap<>(JoystickButtonID.class);
            final Map<JoystickAxisID, Integer> axisBinds = new EnumMap<>(JoystickAxisID.class);

            private final String idf;
            private final boolean canRumble;

            private JoystickVariant(String idf, int[] buttonIDs, int[] axisIDs, boolean canRumble) {
                this.idf = idf;
                this.canRumble = canRumble;
                for (int i = 0; i < JoystickButtonID.values().length; i++) {
                    buttonBinds.put(JoystickButtonID.values()[i], buttonIDs[i]);
                }
                for (int i = 0; i < JoystickAxisID.values().length; i++) {
                    axisBinds.put(JoystickAxisID.values()[i], axisIDs[i]);
                }
            }

            public int getButton(JoystickButtonID id) {
                return buttonBinds.get(id);
            }
            public int getAxis(JoystickAxisID id) {
                return axisBinds.get(id);
            }

            public String getIdf() {
                return idf;
            }

            public boolean canRumble() {
                return canRumble;
            }
        }
    }
}