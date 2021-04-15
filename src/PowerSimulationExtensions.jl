module PowerSimulationExtensions

#################################################################################
# Exports
export MILPDualProblem
export RampReserve
export QuadraticCostRampReserve
export InertiaReserve
export ThermalInertiaBasicUnitCommitment
export ThermalInertiaStandardUnitCommitment
export InertiaFF
export EnergyTargetFF

#################################################################################
# Imports
import PowerSystems
import PowerSimulations
import JuMP
import DataFrames
import ParameterJuMP
import InfrastructureSystems
import Dates
import MathOptInterface
# import Gurobi

const IS = InfrastructureSystems
const MOI = MathOptInterface
const PSY = PowerSystems
const PSI = PowerSimulations
const PM = PowerSimulations.PM
const PJ = ParameterJuMP

#################################################################################
# Includes
include("./core/definitions.jl")

include("device_models/devices/common/add_variable.jl")
include("device_models/devices/common/constraints_structs.jl")
include("device_models/devices/common/rateofchange_constraints.jl")
include("device_models/devices/common/cost_fuctions.jl")
include("device_models/devices/common/commitment_constraint.jl")

include("./core/operations_problem.jl")
include("./service_models/reserves.jl")
include("./core/feedforward.jl")

include("device_models/devices/thermal_generation.jl")

include("./service_models/services_constructor.jl")

end # module
