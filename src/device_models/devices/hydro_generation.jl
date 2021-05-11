abstract type AbstractInertiaUnitCommitment <: PSI.AbstractHydroUnitCommitment end
struct HydroInertiaCommitmentRunOfRiver <: AbstractInertiaUnitCommitment end

function inertia_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.HydroGen, D <: AbstractInertiaUnitCommitment, S <: PM.AbstractPowerModel}
    if _has_inertia_service(model)
        service_model = _get_inertia_service_model(model)
        service = _get_services(first(devices), service_model)[1]

        time_steps = PSI.model_time_steps(optimization_container)
        use_parameters = PSI.model_has_parameters(optimization_container)
        constraint_info = Vector{InertiaServiceConstraintInfo}(undef, length(devices))
        for (idx, d) in enumerate(devices)
            constraint_info[idx] = InertiaCommitmentConstraintInfo(
                PSY.get_name(d),
                nothing,
                _get_inertia(d),
                PSY.get_base_power(d),
            )
            PSI.add_device_services!(constraint_info[idx], d, model)
        end

        hydro_device_inertia!(
            optimization_container,
            constraint_info,
            PSI.make_constraint_name(INERTIA_LIMIT, T),
            PSI.make_variable_name(PSI.ACTIVE_POWER, T),
            PSI.make_variable_name(PSY.get_name(service), typeof(service)),
        )
    end
    return
end
