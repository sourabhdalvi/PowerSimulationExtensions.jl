struct ServiceRampConstraintInfo <: PSI.AbstractRampConstraintInfo
    component_name::String
    ramp_limits::PSI.UpDown
end
