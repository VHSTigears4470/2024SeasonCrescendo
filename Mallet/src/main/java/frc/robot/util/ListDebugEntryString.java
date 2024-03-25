package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ListDebugEntryString {
    private GenericEntry entry = null;

    public ListDebugEntryString(ShuffleboardTab tab, String listTitle,
            String entryName, String defaultValue, BuiltInWidgets widgetType) {
        entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                .add(entryName, defaultValue)
                .withWidget(widgetType)
                .getEntry();
    }

    /**
     * Additional enabled argument for just not creating it to avoid having to make
     * too many changes
     */
    public ListDebugEntryString(ShuffleboardTab tab, String listTitle,
            String entryName, String defaultValue, BuiltInWidgets widgetType, boolean enabled) {
        if (enabled) {
            entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                    .add(entryName, defaultValue)
                    .withWidget(widgetType)
                    .getEntry();
        }
    }

    public void set(String value) {
        if (entry == null) {
            return;
        }
        entry.setString(value);
    }

    public String get(String defaultValue) {
        if (entry == null) {
            return defaultValue;
        }
        String value = entry.get().getString();
        return (value != null) ? value : defaultValue;
    }
}
