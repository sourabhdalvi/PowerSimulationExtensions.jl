struct BookKeepingwInertia <: PSI.AbstractStorageFormulation end

PSI.get_variable_upper_bound(::PSI.EnergyVariable, d::PSY.Storage, ::PSI.AbstractStorageFormulation) = PSY.get_rating(d)

function inertia_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.Storage, D <: BookKeepingwInertia, S <: PM.AbstractPowerModel}
    if _has_inertia_service(model)
        service_model = _get_inertia_service_model(model)
        service = _get_services(first(devices), service_model)[1]

        time_steps = PSI.model_time_steps(optimization_container)
        use_parameters = PSI.model_has_parameters(optimization_container)
        constraint_info = Vector{InertiaServiceConstraintInfo}(undef, length(devices))
        for (idx, d) in enumerate(devices)
            constraint_info[idx] = InertiaServiceConstraintInfo(
                PSY.get_name(d),
                _get_inertia(d),
                PSY.get_output_active_power_limits(d).max,
            )
            PSI.add_device_services!(constraint_info[idx], d, model)
        end

        storage_device_inertia!(
            optimization_container,
            constraint_info,
            (
                PSI.make_constraint_name(INERTIA_LIMIT, T),
                PSI.make_constraint_name(INERTIA_ENERGY_LIMIT, T),
            ),
            (
                PSI.make_variable_name(PSI.ACTIVE_POWER_OUT, T),
                PSI.make_variable_name(PSI.ENERGY, T),
            ),
            PSI.make_variable_name(PSY.get_name(service), typeof(service)),
        )
    end
    return
end
