function PSI.read_variables(optimization_container::PSI.OptimizationContainer)
    var_dict = Dict()
    for (k, v) in PSI.get_variables(optimization_container)
        if length(v.axes) == 1
            var_dict[k] = PSI.axis_array_to_dataframe(v, [:Timesteps])
        else
            var_dict[k] = PSI.axis_array_to_dataframe(v)
        end
    end
    return var_dict
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
