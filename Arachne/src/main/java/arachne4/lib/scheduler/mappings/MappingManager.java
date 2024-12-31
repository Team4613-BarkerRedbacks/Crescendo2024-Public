package arachne4.lib.scheduler.mappings;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;

public class MappingManager<ControllersT> {
    protected final Scheduler scheduler;
    protected final ControllersT controllers;

    public MappingManager(SchedulerProvider schedulerProvider, ControllersT controllers) {
        this(schedulerProvider, controllers, true);
    }

    public MappingManager(SchedulerProvider schedulerProvider, ControllersT controllers, boolean registerMappingsClassInScheduler) {
        this.scheduler = schedulerProvider.getScheduler();
        this.controllers = controllers;

        if (registerMappingsClassInScheduler) scheduler.registerMappingsManager(this);
    }

    protected ButtonMapping mapping(BooleanSupplier button) {
        return scheduler.mapping(button);
    }

    protected AxisMapping mapping(DoubleSupplier axis) {
        return scheduler.mapping(axis);
    }

    protected <DataT> ComplexMapping<DataT> mapping(Supplier<DataT> dataProvider) {
        return scheduler.mapping(dataProvider);
    }
}
