module PowerSimulationExtensions

#################################################################################
# Exports
export MILPDualProblem
export RampReserve
export QuadraticCostRampReserve

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
const PJ = ParameterJuMP

#################################################################################
# Includes
include("device_models/devices/common/add_variable.jl")
include("device_models/devices/common/constraints_structs.jl")
include("device_models/devices/common/rateofchange_constraints.jl")
include("device_models/devices/common/cost_fuctions.jl")

include("./core/operations_problem.jl")
include("./service_models/reserves.jl")

include("./service_models/services_constructor.jl")

end # module
