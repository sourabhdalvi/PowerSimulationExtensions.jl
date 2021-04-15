
abstract type AbstractInertiaStandardUnitCommitment <: PSI.AbstractStandardUnitCommitment end
struct ThermalInertiaBasicUnitCommitment <: AbstractInertiaStandardUnitCommitment end
struct ThermalInertiaStandardUnitCommitment <: AbstractInertiaStandardUnitCommitment end

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{<:PSI.VariableType},
    ::Type{T},
    ::Type{<:PSI.AbstractThermalFormulation},
    ::Type{<:PM.AbstractPowerModel},
    feedforward::InertiaFF,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {T <: PSY.ThermalGen}
    return PSI.DeviceRangeConstraintSpec()
end

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{PSI.ReactivePowerVariable},
    ::Type{T},
    ::Type{<:PSI.AbstractThermalDispatchFormulation},
    ::Type{<:PM.AbstractPowerModel},
    feedforward::InertiaFF,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {T <: PSY.ThermalGen}
    return PSI.DeviceRangeConstraintSpec()
end

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{PSI.ReactivePowerVariable},
    ::Type{T},
    ::Type{<:PSI.AbstractThermalUnitCommitment},
    ::Type{<:PM.AbstractPowerModel},
    feedforward::InertiaFF,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {T <: PSY.ThermalGen}
    return PSI.DeviceRangeConstraintSpec()
end

function _has_inertia_service(model)
    for service_model in PSI.get_services(model)
        if service_model.formulation == InertiaReserve
            return true
        end
    end
    return false
end

function _get_inertia_service_model(model)
    for service_model in PSI.get_services(model)
        if service_model.formulation == InertiaReserve
            return service_model
        end
    end
end

function _get_services(device, service_model)
    services = [s for s in PSY.get_services(device) if isa(s, service_model.component_type)]
    @assert !isempty(services)
    return services
end

function PSI.commitment_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    D <: AbstractInertiaStandardUnitCommitment,
    S <: PM.AbstractPowerModel,
}
    if _has_inertia_service(model)
        service_model = _get_inertia_service_model(model)
        service = _get_services(first(devices), service_model)[1]
        constraint_info = Vector{InertiaCommitmentConstraintInfo}(undef, length(devices))
        initial_conditions = PSI.get_initial_conditions(
            optimization_container,
            PSI.ICKey(PSI.DeviceStatus, T),
        )
        for (idx, ic) in enumerate(initial_conditions)
            constraint_info[idx] = InertiaCommitmentConstraintInfo(
                PSY.get_name(ic.device),
                ic,
                _get_inertia(ic.device),
                PSY.get_base_power(ic.device),
            )
        end
        device_commitment_inertia!(
            optimization_container,
            constraint_info,
            PSI.make_constraint_name(PSI.COMMITMENT, T),
            (
                PSI.make_variable_name(PSI.StartVariable, T),
                PSI.make_variable_name(PSI.StopVariable, T),
                PSI.make_variable_name(PSI.OnVariable, T),
            ),
            PSI.make_variable_name(PSY.get_name(service), typeof(service)),
        )

    else
        PSI.device_commitment!(
            optimization_container,
            PSI.get_initial_conditions(
                optimization_container,
                PSI.ICKey(PSI.DeviceStatus, T),
            ),
            PSI.make_constraint_name(PSI.COMMITMENT, T),
            (
                PSI.make_variable_name(PSI.StartVariable, T),
                PSI.make_variable_name(PSI.StopVariable, T),
                PSI.make_variable_name(PSI.OnVariable, T),
            ),
        )
    end

    return
end
