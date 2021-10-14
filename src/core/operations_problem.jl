abstract type MILPDualProblem <: PSI.PowerSimulationsOperationsProblem end

function PSI.ProblemResults(problem::PSI.OperationsProblem{MILPDualProblem})
    status = PSI.get_run_status(problem)
    status != PSI.RunStatus.SUCCESSFUL &&
        error("problem was not solved successfully: $status")

    container = PSI.get_optimization_container(problem)
    variables = PSI.read_variables(container)

    @warn "Problem is a MILP, the problem will be resolved as LP with  binary variables fixed"

    PSI._apply_warm_start!(problem)
    binary_variables = get_binary_variables(problem)
    fix_binary_variables(binary_variables)
    relax_binary_variables(binary_variables)
    # undo_relax = JuMP.relax_integrality(container.JuMPmodel)

    PSI.solve!(problem)
    duals = PSI.read_duals(PSI.get_optimization_container(problem))
    PSI._apply_warm_start!(problem)

    # undo_relax()
    reset_binary_variables(binary_variables)
    unfix_binary_variables(binary_variables)
    PSI.solve!(problem)

    parameters = PSI.read_parameters(container)
    timestamps = PSI.get_timestamps(problem)
    optimizer_stats = PSI.OptimizerStats(problem)

    for df in Iterators.flatten(((values(variables), values(duals), values(parameters))))
        DataFrames.insertcols!(df, 1, :DateTime => timestamps)
    end

    return PSI.ProblemResults(
        PSI.get_problem_base_power(problem),
        timestamps,
        problem.sys,
        variables,
        duals,
        parameters,
        optimizer_stats,
        mkpath(joinpath(PSI.get_output_dir(problem), "results")),
    )
end

function PSI.write_model_results!(
    store,
    problem::PSI.OperationsProblem{MILPDualProblem},
    timestamp;
    exports = nothing,
)
    optimization_container = PSI.get_optimization_container(problem)
    if exports !== nothing
        export_params = Dict{Symbol, Any}(
            :exports => exports,
            :exports_path => joinpath(exports.path, PSI.get_name(problem)),
            :file_type => PSI.get_export_file_type(exports),
            :resolution => PSI.get_resolution(problem),
            :horizon => PSI.get_horizon(PSI.get_settings(problem)),
        )
    else
        export_params = nothing
    end

    # This line should only be called if the problem is exporting duals. Otherwise ignore.
    if has_binary_variables(PSI.get_optimization_container(problem)) ||
       has_integer_variables(PSI.get_optimization_container(problem))
        @warn "Problem $(PSI.get_simulation_info(problem).name) is a MILP, the problem will be resolved as LP with  binary variables fixed"

        PSI._apply_warm_start!(problem)

        binary_variables = get_binary_variables(problem)
        fix_binary_variables(binary_variables)
        relax_binary_variables(binary_variables)

        PSI.solve!(problem)
        PSI._write_model_dual_results!(
            store,
            optimization_container,
            problem,
            timestamp,
            export_params,
        )
        PSI._apply_warm_start!(problem)
        reset_binary_variables(binary_variables)
        unfix_binary_variables(binary_variables)

        PSI.solve!(problem)
    else
        PSI._write_model_dual_results!(
            store,
            optimization_container,
            problem,
            timestamp,
            export_params,
        )
    end

    PSI._write_model_parameter_results!(
        store,
        optimization_container,
        problem,
        timestamp,
        export_params,
    )
    PSI._write_model_variable_results!(
        store,
        optimization_container,
        problem,
        timestamp,
        export_params,
    )

    return
end

function get_binary_variables(problem)
    binary_variables = Dict()
    variable_refs = PSI.get_variables(problem)
    for (var_name, data_array) in variable_refs
        if isa(data_arrayy, JuMP.Containers.SparseAxisArray)
            if JuMP.is_binary(first(eachindex(v)))
                binary_variables[var_name] = data_array
            end
        else
            if JuMP.is_binary(first(data_array))
                binary_variables[var_name] = data_array
            end
        end
    end
    return binary_variables
end

function relax_binary_variables(binary_variables)
    for (var_name, data_array) in binary_variables
        for var_ref in data_array
            JuMP.unset_binary(var_ref)
        end
    end
    return
end

function fix_binary_variables(binary_variables)
    for (var_name, data_array) in binary_variables
        for var_ref in data_array
            JuMP.fix(var_ref, abs(round(JuMP.value(var_ref))); force = true)
        end
    end
    return
end

function fix_all_variables(optimization_container)
    all_vars = JuMP.all_variables(optimization_container.JuMPmodel)
    JuMP.fix.(all_vars, JuMP.value.(all_vars); force = true)
    return
end

function reset_binary_variables(binary_variables)
    for (var_name, data_array) in binary_variables
        for var_ref in data_array
            JuMP.set_binary(var_ref)
        end
    end
    return
end

function unfix_binary_variables(binary_variables)
    for (var_name, data_array) in binary_variables
        for var_ref in data_array
            JuMP.unfix(var_ref)
        end
    end
    return
end

function unfix_all_variables(optimization_container)
    all_vars = JuMP.all_variables(optimization_container.JuMPmodel)
    JuMP.unfix.(all_vars)
    return
end

function has_binary_variables(container)
    if !isempty(
        JuMP.all_constraints(container.JuMPmodel, JuMP.VariableRef, JuMP.MOI.ZeroOne),
    )
        return true
    end
    return false
end

function has_integer_variables(container)
    if !isempty(
        JuMP.all_constraints(container.JuMPmodel, JuMP.VariableRef, JuMP.MOI.Integer),
    )
        return true
    end
    return false
end

function apply_warm_start(problem)
    optimization_container = PSI.get_optimization_container(problem)
    jump_model = PSI.get_jump_model(optimization_container)
    all_vars = JuMP.all_variables(jump_model)
    JuMP.set_start_value.(all_vars, round.(JuMP.value.(all_vars), digits = 5))
    return
end

#=
# Only to be used with Gurobi Only
function apply_var_hint(problem)
    optimization_container = PSI.get_optimization_container(problem)
    jump_model = PSI.get_jump_model(optimization_container)
    all_vars = JuMP.all_variables(jump_model)
    values_for_the_variables = JuMP.value.(all_vars)

    MOI.set.(
        jump_model,
        Gurobi.VariableAttribute("VarHintVal"),
        all_vars,
        values_for_the_variables,
    )
    return
end
=#
