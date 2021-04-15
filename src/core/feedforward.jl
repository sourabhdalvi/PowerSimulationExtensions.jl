struct InertiaFF <: PSI.AbstractAffectFeedForward
    binary_source_problem::Symbol
    affected_variables::Vector{Symbol}
    service::PSY.Reserve
    cache::Union{Nothing, Type{<:PSI.AbstractCache}}
    function InertiaFF(
        binary_source_problem::AbstractString,
        affected_variables::Vector{<:AbstractString},
        service::PSY.Reserve,
        cache::Union{Nothing, Type{<:PSI.AbstractCache}},
    )
        new(Symbol(binary_source_problem), Symbol.(affected_variables), service, cache)
    end
end

function InertiaFF(; binary_source_problem, affected_variables, service)
    return InertiaFF(binary_source_problem, affected_variables, service, nothing)
end

PSI.get_binary_source_problem(p::InertiaFF) = p.binary_source_problem

struct EnergyTargetFF <: PSI.AbstractAffectFeedForward
    variable_source_problem::Symbol
    affected_variables::Vector{Symbol}
    target_period::Int
    penalty_cost::Float64
    cache::Union{Nothing, Type{<:PSI.AbstractCache}}
    function EnergyTargetFF(
        variable_source_problem::AbstractString,
        affected_variables::Vector{<:AbstractString},
        target_period::Int,
        penalty_cost::Float64,
        cache::Union{Nothing, Type{<:PSI.AbstractCache}},
    )
        new(
            Symbol(variable_source_problem),
            Symbol.(affected_variables),
            target_period,
            penalty_cost,
            cache,
        )
    end
end

function EnergyTargetFF(;
    variable_source_problem,
    affected_variables,
    target_period,
    penalty_cost,
)
    return EnergyTargetFF(
        variable_source_problem,
        affected_variables,
        target_period,
        penalty_cost,
        nothing,
    )
end

get_variable_source_problem(p::EnergyTargetFF) = p.variable_source_problem

####################### Feed Forward Affects ###############################################
@doc raw"""

"""
function inertia_ff(
    optimization_container::PSI.OptimizationContainer,
    cons_name::Symbol,
    constraint_infos::Vector{InertiaServiceConstraintInfo},
    param_reference::PSI.UpdateRef,
    var_name::Symbol,
)
    time_steps = PSI.model_time_steps(optimization_container)
    variable = PSI.get_variable(optimization_container, var_name)

    axes = JuMP.axes(variable)
    set_name = axes[1]
    @assert axes[2] == time_steps
    container = PSI.get_parameter_container(optimization_container, param_reference)
    param_ub = PSI.get_parameter_array(container)
    con_ub =
        PSI.add_cons_container!(optimization_container, cons_name, set_name, time_steps)

    for constraint_info in constraint_infos
        name = PSI.get_component_name(constraint_info)
        for t in time_steps
            con_ub[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                variable[name, t] <= param_ub[name] * PSI.M_VALUE
            )
        end
    end
    return
end

@doc raw"""

"""
function energy_target_ff(
    optimization_container::PSI.OptimizationContainer,
    cons_name::Symbol,
    param_reference::PSI.UpdateRef,
    var_name::Tuple{Symbol, Symbol},
    target_period::Int,
    penalty_cost::Float64,
)
    time_steps = PSI.model_time_steps(optimization_container)
    variable = PSI.get_variable(optimization_container, var_name[1])
    varslack = PSI.get_variable(optimization_container, var_name[2])
    axes = JuMP.axes(variable)
    set_name = axes[1]

    @assert axes[2] == time_steps
    container_ub =
        PSI.add_param_container!(optimization_container, param_reference, set_name)
    param_ub = PSI.get_parameter_array(container_ub)
    multiplier_ub = PSI.get_multiplier_array(container_ub)
    con_ub = PSI.add_cons_container!(optimization_container, cons_name, set_name)

    for name in axes[1]
        value = JuMP.upper_bound(variable[name, 1])
        param_ub[name] = PSI.add_parameter(optimization_container.JuMPmodel, value)
        # default set to 1.0, as this implementation doesn't use multiplier
        multiplier_ub[name] = 1.0
        con_ub[name] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            variable[name, target_period] + varslack[name, target_period] >=
            param_ub[name] * multiplier_ub[name]
        )
        PSI.linear_gen_cost!(
            optimization_container,
            var_name[2],
            name,
            penalty_cost,
            target_period,
        )
    end
end

########################## FeedForward Constraints #########################################

function PSI.feedforward!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, <:PSI.AbstractDeviceFormulation},
    ff_model::InertiaFF,
) where {T <: PSY.StaticInjection}
    bin_var = PSI.make_variable_name(PSI.get_binary_source_problem(ff_model), T)
    parameter_ref = PSI.UpdateRef{JuMP.VariableRef}(bin_var)
    constraint_infos = Vector{PSI.DeviceRangeConstraintInfo}(undef, length(devices))
    inertia_constraint_infos = Vector{InertiaServiceConstraintInfo}(undef, length(devices))
    for (ix, d) in enumerate(devices)
        name = PSY.get_name(d)
        inertia = _get_inertia(d)
        device_base = PSY.get_base_power(d)
        limits = PSY.get_active_power_limits(d)
        constraint_info = PSI.DeviceRangeConstraintInfo(name, limits)
        PSI.add_device_services!(constraint_info, d, model)
        constraint_infos[ix] = constraint_info
        inertia_constraint_infos[ix] =
            InertiaServiceConstraintInfo(name, inertia, device_base)
    end
    for prefix in PSI.get_affected_variables(ff_model)
        var_name = PSI.make_variable_name(prefix, T)
        PSI.semicontinuousrange_ff(
            optimization_container,
            PSI.make_constraint_name(PSI.FEEDFORWARD_BIN, T),
            constraint_infos,
            parameter_ref,
            var_name,
        )
    end
    var_name =
        PSI.make_variable_name(PSY.get_name(ff_model.service), typeof(ff_model.service))
    inertia_ff(
        optimization_container,
        PSI.make_constraint_name(INERTIA_LIMIT, T),
        inertia_constraint_infos,
        parameter_ref,
        var_name,
    )
end

function feedforward!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::PSI.DeviceModel{T, D},
    ff_model::EnergyTargetFF,
) where {T <: PSY.StaticInjection, D <: PSI.AbstractDeviceFormulation}
    PSI.add_variables!(optimization_container, PSI.EnergyShortageVariable, devices, D())
    for prefix in PSI.get_affected_variables(ff_model)
        var_name = PSI.make_variable_name(prefix, T)
        varslack_name = PSI.make_variable_name(PSI.ENERGY_SHORTAGE, T)
        parameter_ref = PSI.UpdateRef{JuMP.VariableRef}(var_name)
        energy_target_ff(
            optimization_container,
            PSI.make_constraint_name(FEEDFORWARD_ENERGY_TARGET, T),
            parameter_ref,
            (var_name, varslack_name),
            ff_model.target_period,
            ff_model.penalty_cost,
        )
    end
end
