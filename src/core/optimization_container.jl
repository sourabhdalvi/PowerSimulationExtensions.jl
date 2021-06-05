function PSI.read_variables(optimization_container::PSI.OptimizationContainer)
    var_dict = Dict()
    for (k, v) in PSI.get_variables(optimization_container)
        if length(v.axes) == 1
            var_dict[k] =  PSI.axis_array_to_dataframe(v, [:Timesteps])
        else
            var_dict[k] =  PSI.axis_array_to_dataframe(v)
        end
    end
    return var_dict
end
