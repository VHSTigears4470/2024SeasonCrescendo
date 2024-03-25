package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ListDebugEntryDouble {
    private GenericEntry entry = null;

    public ListDebugEntryDouble(ShuffleboardTab tab, String listTitle,
            String entryName, Double defaultValue, BuiltInWidgets widgetType) {
        entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                .add(entryName, defaultValue)
                .withWidget(widgetType)
                .getEntry();
    }

    /**
     * Additional enabled argument for just not creating it to avoid having to make
     * too many changes
     */
    public ListDebugEntryDouble(ShuffleboardTab tab, String listTitle,
            String entryName, Double defaultValue, BuiltInWidgets widgetType, boolean enabled) {
        if (enabled) {
            entry = tab.getLayout(listTitle, BuiltInLayouts.kList)
                    .add(entryName, defaultValue)
                    .withWidget(widgetType)
                    .getEntry();
        }
    }

    public void set(Double value) {
        if (entry == null) {
            return;
        }
        entry.setDouble(value);
    }

    public Double get(Double defaultValue) {
        if (entry == null) {
            return defaultValue;
        }
        Double value = entry.get().getDouble();
        return (value != null) ? value : defaultValue;
    }
}
