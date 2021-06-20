
function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, BookKeepingwInertia},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReserveVariable,
        devices,
        BookKeepingwInertia(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, BookKeepingwInertia())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Energy Balanace limits
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    inertia_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, BookKeepingwInertia},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerInVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerOutVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        BookKeepingwInertia(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReserveVariable,
        devices,
        BookKeepingwInertia(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, BookKeepingwInertia())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    inertia_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Energy Balanace limits
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, SimpleBatteryDispatch},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        SimpleBatteryDispatch(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.ReactivePowerVariable,
        devices,
        SimpleBatteryDispatch(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        SimpleBatteryDispatch(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, SimpleBatteryDispatch())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Energy Balanace limits
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end

function PSI.construct_device!(
    optimization_container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.DeviceModel{St, SimpleBatteryDispatch},
    ::Type{S},
) where {St <: PSY.Storage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(St, sys)

    if !PSI.validate_available_devices(St, devices)
        return
    end

    # Variables
    PSI.add_variables!(
        optimization_container,
        PSI.ActivePowerVariable,
        devices,
        SimpleBatteryDispatch(),
    )
    PSI.add_variables!(
        optimization_container,
        PSI.EnergyVariable,
        devices,
        BookKeepingwInertia(),
    )

    # Initial Conditions
    PSI.initial_conditions!(optimization_container, devices, SimpleBatteryDispatch())

    # Constraints
    PSI.add_constraints!(
        optimization_container,
        PSI.RangeConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.energy_capacity_constraints!(
        optimization_container,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )
    PSI.feedforward!(optimization_container, devices, model, PSI.get_feedforward(model))

    # Energy Balanace limits
    PSI.add_constraints!(
        optimization_container,
        PSI.EnergyBalanceConstraint,
        PSI.EnergyVariable,
        devices,
        model,
        S,
        PSI.get_feedforward(model),
    )

    return
end
