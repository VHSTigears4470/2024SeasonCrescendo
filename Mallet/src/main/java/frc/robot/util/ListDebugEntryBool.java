package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ListDebugEntryBool {
    private GenericEntry entry = null;

    public ListDebugEntryBool(ShuffleboardTab tab, String listTitle,
            String entryName, Boolean defaultValue, BuiltInWidgets widgetType) {
        entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                .add(entryName, defaultValue)
                .withWidget(widgetType)
                .getEntry();
    }

    /**
     * Additional enabled argument for just not creating it to avoid having to make
     * too many changes
     */
    public ListDebugEntryBool(ShuffleboardTab tab, String listTitle,
            String entryName, Boolean defaultValue, BuiltInWidgets widgetType, boolean enabled) {
        if (enabled) {
            entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                    .add(entryName, defaultValue)
                    .withWidget(widgetType)
                    .getEntry();
        }
    }

    public void set(Boolean value) {
        if (entry == null) {
            return;
        }
        entry.setBoolean(value);
    }

    public Boolean get(Boolean defaultValue) {
        if (entry == null) {
            return defaultValue;
        }
        Boolean value = entry.get().getBoolean();
        return (value != null) ? value : defaultValue;
    }
}
