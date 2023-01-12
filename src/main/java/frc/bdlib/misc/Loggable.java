package frc.bdlib.misc;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class Loggable<T> implements BDUpdatable {
    public static NetworkTable parent = NetworkTableInstance.getDefault().getTable("Loggables");

    public static record SubsystemTable(String name) {
        static NetworkTable table;

        public SubsystemTable init() {
            table = parent.getSubTable(this.name());
            return this;
        }

        public NetworkTable table() {
            return table;
        }

        public <T> void addLoggable(String name, Supplier<T> supplier) {
            Loggable.of(this, name, supplier);
        }
    };

    public static SubsystemTable uncategorizedTable = new SubsystemTable("Uncategorized").init();

    public static <T> void of(String name, Supplier<T> supplier) {
        Loggable.of(uncategorizedTable, name, supplier);
    }

    private static <T> void of(SubsystemTable category, String name, Supplier<T> supplier) {
        new Loggable<T>(category, name, supplier);
    }

    public static SubsystemTable subsystemOf(String name) {
        return new SubsystemTable(name).init();
    }

    private Supplier<T> loggableValueSupplier;
    private GenericPublisher valueTopic;

    private Loggable(SubsystemTable table, String name, Supplier<T> loggableValueSupplier) {
        this.loggableValueSupplier = loggableValueSupplier;
        this.valueTopic = 
            table.table().getTopic(name)
            .genericPublish(
                loggableValueSupplier.getClass().getTypeName(), 
                PubSubOption.sendAll(false)
            );

        BDManager.getInstance().register(this, false);
    }

    @Override
    public void update() {
        valueTopic.setValue(loggableValueSupplier.get());
    }

    @Override
    public String getStatus() {
        return "OK";
    }

    @Override
    public String getID() {
        return "K";
    }
}
