package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ListDebugEntry {
    private GenericEntry entry = null;

    public ListDebugEntry(ShuffleboardTab tab, String listTitle,
            String entryName, Object defaultValue, BuiltInWidgets widgetType) {
        entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                .add(entryName, defaultValue)
                .withWidget(widgetType)
                .getEntry();
    }

    /**
     * Additional enabled argument for just not creating it to avoid having to make
     * too many changes
     */
    public ListDebugEntry(ShuffleboardTab tab, String listTitle,
            String entryName, Object defaultValue, BuiltInWidgets widgetType, boolean enabled) {
        if (enabled) {
            entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                    .add(entryName, defaultValue)
                    .withWidget(widgetType)
                    .getEntry();
        }
    }

    public void set(Object value) {
        if (value == null) {
            return;
        }
        entry.setValue(value);
    }

    public Object get(Object defaultValue) {
        if (entry == null) {
            return defaultValue;
        }
        Object value = entry.get().getValue();
        return (value != null) ? value : defaultValue;
    }
}
