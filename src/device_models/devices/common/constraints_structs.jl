abstract type AbstractServiceConstraintInfo <: PSI.AbstractConstraintInfo end

struct ServiceRampConstraintInfo <: PSI.AbstractRampConstraintInfo
    component_name::String
    ramp_limits::PSI.UpDown
end

struct InertiaServiceConstraintInfo <: AbstractServiceConstraintInfo
    component_name::String
    inertia::Float64
    base_power::Float64
end

struct InertiaCommitmentConstraintInfo <: AbstractServiceConstraintInfo
    component_name::String
    ic_power::PSI.InitialCondition
    inertia::Float64
    base_power::Float64
end
